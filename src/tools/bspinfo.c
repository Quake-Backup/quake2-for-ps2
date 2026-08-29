/* ================================================================================================
 * -*- C -*-
 * File: bspinfo.c
 * Created on: 27/08/26
 * Brief: Reports the largest value each BSP lump reaches across a set of maps, next to the
 *        MAX_MAP_* cap the engine reserves a static array for.
 *
 * The PS2 port sizes cmodel.c's collision arrays to what the shipped maps actually use rather
 * than to id's design bounds (see the note above the MAX_MAP_* block in src/common/q_files.h).
 * Those caps are tight, so this tool exists to re-derive them: point it at a pak or a directory
 * of .bsp files and it prints the worst case per lump, which map set it, and how much slack is
 * left over the cap. Anything at or over 100% would fail to load with a Com_Error.
 *
 *   build/tools/bspinfo baseq2/pak0.pak
 *   build/tools/bspinfo baseq2/pak0/maps
 *
 * This source code is released under the GNU GPL v2 license.
 * Check the accompanying LICENSE file for details.
 * ================================================================================================ */

#include <dirent.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

/*
 * From Quake2:
 */

// 4CC 'PACK'
#define PAK_HEADER_IDENT (('K' << 24) + ('C' << 16) + ('A' << 8) + 'P')

// 4CC 'IBSP'
#define BSP_HEADER_IDENT (('P' << 24) + ('S' << 16) + ('B' << 8) + 'I')

#define BSP_VERSION   38
#define BSP_NUM_LUMPS 19

typedef struct
{
    char name[56];
    int filepos;
    int filelen;
} pak_file_t;

typedef struct
{
    int ident;
    int dirofs;
    int dirlen;
} pak_header_t;

typedef struct
{
    int fileofs;
    int filelen;
} bsp_lump_t;

typedef struct
{
    int ident;
    int version;
    bsp_lump_t lumps[BSP_NUM_LUMPS];
} bsp_header_t;

/*
 * Lump table:
 *
 * 'elem_size' is the on-disk size of one element, so filelen/elem_size is the count the engine
 * bounds-checks. The byte lumps (entities, visibility, lighting) use 1 and are counted in bytes.
 * 'cap' mirrors the MAX_MAP_* value in src/common/q_files.h; keep the two in sync. A cap of 0
 * means the engine does not bound that lump with a constant of its own.
 */
typedef struct
{
    const char * name;
    int elem_size;
    long cap;
    const char * cap_name;
    bool reserves_array; // true when cmodel.c sizes a static array with the cap
} lump_info_t;

static const lump_info_t lump_info[BSP_NUM_LUMPS] = {
    /*  0 */ { "ENTITIES",    1,  0x28000, "MAX_MAP_ENTSTRING",   true  },
    /*  1 */ { "PLANES",      20, 16384,   "MAX_MAP_PLANES",      true  },
    /*  2 */ { "VERTEXES",    12, 65536,   "MAX_MAP_VERTS",       false },
    /*  3 */ { "VISIBILITY",  1,  0x70000, "MAX_MAP_VISIBILITY",  true  },
    /*  4 */ { "NODES",       28, 12288,   "MAX_MAP_NODES",       true  },
    /*  5 */ { "TEXINFO",     76, 1536,    "MAX_MAP_TEXINFO",     true  },
    /*  6 */ { "FACES",       20, 65536,   "MAX_MAP_FACES",       false },
    /*  7 */ { "LIGHTING",    1,  0x200000,"MAX_MAP_LIGHTING",    false },
    /*  8 */ { "LEAFS",       28, 12288,   "MAX_MAP_LEAFS",       true  },
    /*  9 */ { "LEAFFACES",   2,  65536,   "MAX_MAP_LEAFFACES",   false },
    /* 10 */ { "LEAFBRUSHES", 2,  16384,   "MAX_MAP_LEAFBRUSHES", true  },
    /* 11 */ { "EDGES",       4,  128000,  "MAX_MAP_EDGES",       false },
    /* 12 */ { "SURFEDGES",   4,  256000,  "MAX_MAP_SURFEDGES",   false },
    /* 13 */ { "MODELS",      48, 224,     "MAX_MAP_MODELS",      true  },
    /* 14 */ { "BRUSHES",     12, 6144,    "MAX_MAP_BRUSHES",     true  },
    /* 15 */ { "BRUSHSIDES",  4,  40960,   "MAX_MAP_BRUSHSIDES",  true  },
    /* 16 */ { "POP",         1,  0,       "",                    false },
    /* 17 */ { "AREAS",       8,  256,     "MAX_MAP_AREAS",       true  },
    /* 18 */ { "AREAPORTALS", 8,  64,      "MAX_MAP_AREAPORTALS", true  },
};

typedef struct
{
    long count;
    char map[64];
} lump_max_t;

static lump_max_t maxima[BSP_NUM_LUMPS];
static int num_maps_scanned;

/*
 * Scanning:
 */

// Folds one map's header into the running maxima. 'data' is the first sizeof(bsp_header_t)
// bytes of the BSP; anything that isn't an IBSP v38 header is skipped with a warning.
static void account_bsp(const char * map_name, const unsigned char * data, size_t data_len)
{
    bsp_header_t hdr;
    int i;

    if (data_len < sizeof(hdr))
    {
        fprintf(stderr, "warning: '%s' is too small to hold a BSP header, skipping.\n", map_name);
        return;
    }

    memcpy(&hdr, data, sizeof(hdr));

    if (hdr.ident != BSP_HEADER_IDENT)
    {
        fprintf(stderr, "warning: '%s' is not an IBSP file, skipping.\n", map_name);
        return;
    }
    if (hdr.version != BSP_VERSION)
    {
        fprintf(stderr, "warning: '%s' is BSP version %d, expected %d, skipping.\n",
                map_name, hdr.version, BSP_VERSION);
        return;
    }

    for (i = 0; i < BSP_NUM_LUMPS; ++i)
    {
        long count = (long)hdr.lumps[i].filelen / lump_info[i].elem_size;
        if (count > maxima[i].count)
        {
            maxima[i].count = count;
            snprintf(maxima[i].map, sizeof(maxima[i].map), "%s", map_name);
        }
    }

    ++num_maps_scanned;
}

// Strips directories off a path so the report shows 'lab.bsp' rather than 'maps/lab.bsp'.
static const char * base_name(const char * path)
{
    const char * slash = strrchr(path, '/');
    return slash ? (slash + 1) : path;
}

static bool scan_pak(const char * pak_path)
{
    pak_header_t hdr;
    pak_file_t * dir;
    int num_files, i;
    FILE * fp = fopen(pak_path, "rb");

    if (fp == NULL)
    {
        fprintf(stderr, "error: unable to open '%s'.\n", pak_path);
        return false;
    }

    if (fread(&hdr, sizeof(hdr), 1, fp) != 1 || hdr.ident != PAK_HEADER_IDENT)
    {
        fprintf(stderr, "error: '%s' is not a Quake 2 PAK archive.\n", pak_path);
        fclose(fp);
        return false;
    }

    num_files = hdr.dirlen / (int)sizeof(pak_file_t);
    if (num_files <= 0)
    {
        fprintf(stderr, "error: '%s' has an empty directory.\n", pak_path);
        fclose(fp);
        return false;
    }

    dir = (pak_file_t *)malloc((size_t)num_files * sizeof(pak_file_t));
    if (dir == NULL)
    {
        fprintf(stderr, "error: out of memory reading the PAK directory.\n");
        fclose(fp);
        return false;
    }

    if (fseek(fp, hdr.dirofs, SEEK_SET) != 0 ||
        fread(dir, sizeof(pak_file_t), (size_t)num_files, fp) != (size_t)num_files)
    {
        fprintf(stderr, "error: truncated PAK directory in '%s'.\n", pak_path);
        free(dir);
        fclose(fp);
        return false;
    }

    for (i = 0; i < num_files; ++i)
    {
        unsigned char header_bytes[sizeof(bsp_header_t)];
        const char * name = dir[i].name;
        size_t name_len = strlen(name);

        if (name_len < 4 || strcasecmp(name + name_len - 4, ".bsp") != 0)
        {
            continue;
        }

        // Only the header is needed, so seek to each map rather than reading the whole lump set.
        if (fseek(fp, dir[i].filepos, SEEK_SET) != 0 ||
            fread(header_bytes, 1, sizeof(header_bytes), fp) != sizeof(header_bytes))
        {
            fprintf(stderr, "warning: unable to read the header of '%s', skipping.\n", name);
            continue;
        }

        account_bsp(base_name(name), header_bytes, sizeof(header_bytes));
    }

    free(dir);
    fclose(fp);
    return true;
}

static bool scan_directory(const char * dir_path)
{
    struct dirent * ent;
    DIR * dir = opendir(dir_path);

    if (dir == NULL)
    {
        fprintf(stderr, "error: unable to open directory '%s'.\n", dir_path);
        return false;
    }

    while ((ent = readdir(dir)) != NULL)
    {
        unsigned char header_bytes[sizeof(bsp_header_t)];
        char full_path[1024];
        size_t name_len = strlen(ent->d_name);
        FILE * fp;

        if (name_len < 4 || strcasecmp(ent->d_name + name_len - 4, ".bsp") != 0)
        {
            continue;
        }

        snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, ent->d_name);

        fp = fopen(full_path, "rb");
        if (fp == NULL)
        {
            fprintf(stderr, "warning: unable to open '%s', skipping.\n", full_path);
            continue;
        }

        if (fread(header_bytes, 1, sizeof(header_bytes), fp) == sizeof(header_bytes))
        {
            account_bsp(ent->d_name, header_bytes, sizeof(header_bytes));
        }
        else
        {
            fprintf(stderr, "warning: unable to read the header of '%s', skipping.\n", full_path);
        }

        fclose(fp);
    }

    closedir(dir);
    return true;
}

/*
 * Reporting:
 */

static void print_report()
{
    int i;
    int over_cap = 0;

    printf("\n%d map(s) scanned.\n\n", num_maps_scanned);
    printf("%-13s %10s  %-16s %10s  %6s  %s\n",
           "LUMP", "WORST", "SET BY", "CAP", "USED", "CONSTANT");
    printf("--------------------------------------------------------------------------------\n");

    for (i = 0; i < BSP_NUM_LUMPS; ++i)
    {
        const lump_info_t * info = &lump_info[i];

        printf("%-13s %10ld  %-16s ", info->name, maxima[i].count,
               maxima[i].count > 0 ? maxima[i].map : "-");

        if (info->cap > 0)
        {
            double used = 100.0 * (double)maxima[i].count / (double)info->cap;
            printf("%10ld  %5.1f%%  %s%s\n", info->cap, used, info->cap_name,
                   info->reserves_array ? "" : " (bound only)");

            if (maxima[i].count > info->cap)
            {
                ++over_cap;
            }
        }
        else
        {
            printf("%10s  %6s  %s\n", "-", "-", "unbounded");
        }
    }

    printf("--------------------------------------------------------------------------------\n");
    printf("'USED' is the worst map as a percentage of the cap; at 100%% or above the map\n");
    printf("fails to load with a Com_Error. Lumps marked '(bound only)' are validated but\n");
    printf("do not reserve a static array, so lowering them saves nothing.\n");

    if (over_cap > 0)
    {
        printf("\n*** %d lump(s) EXCEED their cap - raise them in src/common/q_files.h ***\n", over_cap);
    }
}

int main(int argc, const char * argv[])
{
    struct stat path_stat;
    const char * path;
    bool ok;

    if (argc != 2)
    {
        printf("Usage:\n  %s <pak-file | directory-of-bsps>\n\n", argv[0]);
        printf("Reports the largest value each BSP lump reaches across every map found,\n");
        printf("next to the MAX_MAP_* cap from src/common/q_files.h. Use it to re-derive\n");
        printf("those caps after adding a mission pack or custom maps.\n\n");
        printf("Examples:\n");
        printf("  %s baseq2/pak0.pak\n", argv[0]);
        printf("  %s baseq2/pak0/maps\n", argv[0]);
        return EXIT_FAILURE;
    }

    path = argv[1];

    if (stat(path, &path_stat) != 0)
    {
        fprintf(stderr, "error: '%s' does not exist.\n", path);
        return EXIT_FAILURE;
    }

    ok = S_ISDIR(path_stat.st_mode) ? scan_directory(path) : scan_pak(path);
    if (!ok)
    {
        return EXIT_FAILURE;
    }

    if (num_maps_scanned == 0)
    {
        fprintf(stderr, "error: no usable BSP files found in '%s'.\n", path);
        return EXIT_FAILURE;
    }

    print_report();
    return EXIT_SUCCESS;
}
