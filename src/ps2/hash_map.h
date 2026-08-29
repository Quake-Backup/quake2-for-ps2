#pragma once
/* ================================================================================================
 * File: hash_map.h
 * Brief: Fixed-capacity u64 -> u16 hash map, used by the asset caches to map a name hash
 *        (hash.h) onto a SmallPool slot index (small_pool.h).
 *
 *  This replaces the std::unordered_map the caches used to carry. That container was the
 *  only thing in the whole link that reached into libstdc++, and it dragged ~125 KB of
 *  dead weight behind it: operator new pulls in bad_alloc, which pulls in the exception
 *  machinery, which pulls in __verbose_terminate_handler, which pulls in the C++ name
 *  demangler - all of it unreachable under -fno-exceptions -fno-rtti, and none of it worth
 *  paying for on a 32 MB console to look up a few hundred asset names.
 *
 *  Open addressing with linear probing and tombstones. Storage is inline, so a map is a
 *  plain member of the cache with no allocation at all. Capacity is the next power of two
 *  at or above 1.5x MaxEntries, keeping the load factor under ~67% where linear probing
 *  still finds a slot in a couple of steps.
 *
 *  Keys are FNV-1a hashes, so a collision means two different names landed on the same
 *  64-bit value. That is vanishingly rare but not impossible, and this map cannot tell the
 *  difference - callers verify the hit against the stored name, exactly as they did with
 *  std::unordered_map.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/common.h"
#include <tamtypes.h>

namespace ps2 {

// Smallest power of two >= 'n'. Constexpr so Capacity below is a compile-time constant.
constexpr u32 NextPow2(u32 n)
{
    u32 p = 1;
    while (p < n)
    {
        p <<= 1;
    }
    return p;
}

// Maps at most 'MaxEntries' distinct u64 keys to u16 values.
template<u32 MaxEntries>
class HashMap final
{
public:
    static constexpr u16 kInvalidValue = u16(~0);
    static constexpr u32 kCapacity     = NextPow2(MaxEntries + (MaxEntries / 2u));

    static_assert(MaxEntries > 0 && MaxEntries < 0xFFFF, "HashMap is limited to 64K entries!");
    static_assert(kCapacity > MaxEntries, "HashMap capacity must exceed its entry count!");

    HashMap() = default;

    // Non-copyable, to match the pools these sit beside.
    HashMap(const HashMap &) = delete;
    HashMap & operator=(const HashMap &) = delete;

    u32 Size() const { return m_count; }

    // Returns the stored value, or kInvalidValue when the key is absent.
    u16 Find(const u64 key) const
    {
        u32 i = Slot(key);
        for (u32 probes = 0; probes < kCapacity; ++probes)
        {
            // An empty slot ends the probe run: a matching key would have been
            // placed at or before it. A tombstone does not - the key may have
            // been inserted past a since-removed entry, so keep walking.
            if (m_state[i] == State::kEmpty)
            {
                return kInvalidValue;
            }
            if (m_state[i] == State::kOccupied && m_keys[i] == key)
            {
                return m_values[i];
            }
            i = (i + 1u) & (kCapacity - 1u);
        }
        return kInvalidValue;
    }

    // Inserts key -> value. Returns false (leaving the map untouched) if the key is
    // already present or the map is full; the caller decides whether either is an error.
    bool Insert(const u64 key, const u16 value)
    {
        if (m_count >= MaxEntries)
        {
            return false;
        }

        // Remember the first tombstone seen so the insert reclaims it, but keep
        // probing to the end of the run first - the key may already be present
        // further along, and inserting a duplicate would shadow it forever.
        u32 firstFree = kCapacity;
        u32 i = Slot(key);

        for (u32 probes = 0; probes < kCapacity; ++probes)
        {
            if (m_state[i] == State::kEmpty)
            {
                if (firstFree == kCapacity)
                {
                    firstFree = i;
                }
                break;
            }
            if (m_state[i] == State::kTombstone)
            {
                if (firstFree == kCapacity)
                {
                    firstFree = i;
                }
            }
            else if (m_keys[i] == key)
            {
                return false; // Already present.
            }
            i = (i + 1u) & (kCapacity - 1u);
        }

        if (firstFree == kCapacity)
        {
            return false; // Table is full of live entries and tombstones.
        }

        m_keys[firstFree]   = key;
        m_values[firstFree] = value;
        m_state[firstFree]  = State::kOccupied;
        ++m_count;
        return true;
    }

    // Removes one key. Returns false if it was not present.
    bool Remove(const u64 key)
    {
        u32 i = Slot(key);
        for (u32 probes = 0; probes < kCapacity; ++probes)
        {
            if (m_state[i] == State::kEmpty)
            {
                return false;
            }
            if (m_state[i] == State::kOccupied && m_keys[i] == key)
            {
                m_state[i] = State::kTombstone;
                --m_count;
                return true;
            }
            i = (i + 1u) & (kCapacity - 1u);
        }
        return false;
    }

    // Removes every entry the predicate accepts, and returns how many went.
    // 'shouldRemove' is called once per live entry as bool(u64 key, u16 value);
    // it must not touch the map itself.
    //
    // This is the eviction pass both asset caches run at EndRegistration. It
    // rebuilds rather than tombstoning in place, which keeps a level change from
    // leaving the table full of tombstones for the next one to probe through.
    template<typename Predicate>
    u32 RemoveIf(Predicate && shouldRemove)
    {
        u64 keptKeys[MaxEntries];
        u16 keptValues[MaxEntries];
        u32 keptCount = 0;
        u32 removed   = 0;

        for (u32 i = 0; i < kCapacity; ++i)
        {
            if (m_state[i] != State::kOccupied)
            {
                continue;
            }

            if (shouldRemove(m_keys[i], m_values[i]))
            {
                ++removed;
            }
            else
            {
                keptKeys[keptCount]   = m_keys[i];
                keptValues[keptCount] = m_values[i];
                ++keptCount;
            }
        }

        if (removed > 0)
        {
            Clear();
            for (u32 i = 0; i < keptCount; ++i)
            {
                Insert(keptKeys[i], keptValues[i]);
            }
        }

        return removed;
    }

    void Clear()
    {
        for (u32 i = 0; i < kCapacity; ++i)
        {
            m_state[i] = State::kEmpty;
        }
        m_count = 0;
    }

private:
    enum class State : u8
    {
        kEmpty = 0, // Never written; ends a probe run.
        kOccupied,
        kTombstone, // Was occupied; does not end a probe run.
    };

    static u32 Slot(const u64 key)
    {
        // The keys are already well-mixed FNV-1a hashes, so the low bits make a
        // fine bucket index; the capacity is a power of two, hence the mask.
        return static_cast<u32>(key) & (kCapacity - 1u);
    }

    u32   m_count             = 0;
    u64   m_keys[kCapacity]   = {};
    u16   m_values[kCapacity] = {};
    State m_state[kCapacity]  = {}; // State::kEmpty default
};

} // namespace ps2
