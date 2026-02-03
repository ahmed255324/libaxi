#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define CONFIGFS_MOUNTPOINT "/sys/kernel/config"
#define DT_OVERLAY_ROOT     "/sys/kernel/config/device-tree/overlays"

typedef enum {
    DTO_OK = 0,
    DTO_USAGE = 2,
    DTO_NOT_ROOT = 3,
    DTO_MOUNT_FAIL = 4,
    DTO_CREATE_FAIL = 5,
    DTO_IO_FAIL = 6,
    DTO_APPLY_TIMEOUT = 7,
    DTO_VERIFY_FAIL = 8
} dto_rc_t;

static int path_exists(const char *p) {
    struct stat st;
    return stat(p, &st) == 0;
}

static int is_dir(const char *p) {
    struct stat st;
    if (stat(p, &st) != 0) return 0;
    return S_ISDIR(st.st_mode);
}

static int ensure_dir(const char *p, mode_t mode) {
    if (mkdir(p, mode) == 0) return 0;
    if (errno == EEXIST && is_dir(p)) return 0;
    return -1;
}

static int mount_configfs_if_needed(void) {
    if (!is_dir(CONFIGFS_MOUNTPOINT)) {
        errno = ENOENT;
        return -1;
    }

    // Prüfen ob configfs schon gemountet ist, indem wir device-tree erwarten
    if (is_dir(CONFIGFS_MOUNTPOINT "/device-tree")) {
        return 0;
    }

    if (mount("none", CONFIGFS_MOUNTPOINT, "configfs", 0, "") != 0) {
        return -1;
    }

    // Nach Mount sollte device-tree auftauchen (wenn Kernel-Optionen passen)
    // Manche Systeme erstellen /device-tree erst beim ersten Zugriff, aber i.d.R. existiert es.
    return 0;
}

static int rm_rf_overlay_dir(const char *overlay_name) {
    // configfs: Overlay entfernen = rmdir des Overlay-Verzeichnisses
    char ovl_dir[512];
    snprintf(ovl_dir, sizeof(ovl_dir), "%s/%s", DT_OVERLAY_ROOT, overlay_name);
    if (!path_exists(ovl_dir)) return 0;

    if (rmdir(ovl_dir) != 0) {
        return -1;
    }
    return 0;
}

static int copy_file_to_fd(const char *src_path, int out_fd) {
    int in = open(src_path, O_RDONLY);
    if (in < 0) return -1;

    char buf[64 * 1024];
    ssize_t r;
    while ((r = read(in, buf, sizeof(buf))) > 0) {
        ssize_t off = 0;
        while (off < r) {
            ssize_t w = write(out_fd, buf + off, (size_t)(r - off));
            if (w < 0) { close(in); return -1; }
            off += w;
        }
    }
    if (r < 0) { close(in); return -1; }

    close(in);
    return 0;
}

static int write_dtbo(const char *overlay_name, const char *dtbo_path) {
    char ovl_dir[512];
    snprintf(ovl_dir, sizeof(ovl_dir), "%s/%s", DT_OVERLAY_ROOT, overlay_name);

    if (ensure_dir(DT_OVERLAY_ROOT, 0755) != 0) return -1;
    if (ensure_dir(ovl_dir, 0755) != 0) return -1;

    char dtbo_cfgfs[600];
    snprintf(dtbo_cfgfs, sizeof(dtbo_cfgfs), "%s/dtbo", ovl_dir);

    int out = open(dtbo_cfgfs, O_WRONLY);
    if (out < 0) return -1;

    int rc = copy_file_to_fd(dtbo_path, out);
    int saved = errno;
    close(out);
    errno = saved;

    return rc;
}

static int read_text_file(const char *path, char *out, size_t out_sz) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;

    ssize_t n = read(fd, out, out_sz - 1);
    int saved = errno;
    close(fd);
    errno = saved;

    if (n < 0) return -1;
    out[n] = '\0';
    return 0;
}

static int wait_for_applied(const char *overlay_name, int timeout_ms) {
    char status_path[600];
    snprintf(status_path, sizeof(status_path), "%s/%s/status", DT_OVERLAY_ROOT, overlay_name);

    const int sleep_step_ms = 50;
    int waited = 0;

    while (waited <= timeout_ms) {
        char buf[128];
        if (read_text_file(status_path, buf, sizeof(buf)) == 0) {
            // status enthält meist "applied" oder "unapplied"
            if (strstr(buf, "applied")) return 0;
            if (strstr(buf, "unapplied")) {
                // weiter warten
            }
        } else {
            // status-Datei nicht vorhanden -> manche Kernel/BSPs, weiter warten
        }

        usleep(sleep_step_ms * 1000);
        waited += sleep_step_ms;
    }
    errno = ETIMEDOUT;
    return -1;
}

static int verify_proc_dt_node(const char *proc_dt_path) {
    // proc_dt_path soll relativ zu /proc/device-tree sein, z.B. "amba_pl@0/myip@a0000000"
    // oder auch "/" für root.
    char full[800];
    if (!proc_dt_path || proc_dt_path[0] == '\0') return 0; // keine Prüfung gewünscht

    if (strcmp(proc_dt_path, "/") == 0) {
        snprintf(full, sizeof(full), "/proc/device-tree");
    } else {
        // ohne führenden Slash akzeptieren
        if (proc_dt_path[0] == '/') {
            snprintf(full, sizeof(full), "/proc/device-tree%s", proc_dt_path);
        } else {
            snprintf(full, sizeof(full), "/proc/device-tree/%s", proc_dt_path);
        }
    }

    return access(full, F_OK) == 0 ? 0 : -1;
}

static void print_usage(const char *argv0) {
    fprintf(stderr,
        "Usage:\n"
        "  %s load  <overlay_name> <file.dtbo> [--force] [--timeout-ms N] [--check-node PATH]\n"
        "  %s unload <overlay_name>\n"
        "\n"
        "Options (load):\n"
        "  --force           If overlay dir exists, remove it first\n"
        "  --timeout-ms N    Wait up to N ms for status=applied (default 2000)\n"
        "  --check-node PATH Verify that /proc/device-tree/PATH exists after apply\n"
        "                   Example: --check-node amba_pl@0/myip@a0000000\n",
        argv0, argv0);
}

int test_main(int argc, char **argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return DTO_USAGE;
    }

    // Root check
    if (geteuid() != 0) {
        fprintf(stderr, "Error: must run as root (or with proper privileges).\n");
        return DTO_NOT_ROOT;
    }

    const char *cmd = argv[1];
    const char *overlay_name = argv[2];

    if (mount_configfs_if_needed() != 0) {
        perror("Error: configfs mount failed (kernel configfs/of_configfs?)");
        return DTO_MOUNT_FAIL;
    }

    // ensure overlays root exists
    if (ensure_dir(DT_OVERLAY_ROOT, 0755) != 0) {
        perror("Error: cannot access overlays root");
        return DTO_CREATE_FAIL;
    }

    if (strcmp(cmd, "unload") == 0) {
        if (argc != 3) {
            print_usage(argv[0]);
            return DTO_USAGE;
        }
        if (rm_rf_overlay_dir(overlay_name) != 0) {
            perror("Error: unload failed (rmdir overlay)");
            return DTO_IO_FAIL;
        }
        printf("OK: overlay '%s' unloaded.\n", overlay_name);
        return DTO_OK;
    }

    if (strcmp(cmd, "load") != 0) {
        print_usage(argv[0]);
        return DTO_USAGE;
    }

    if (argc < 4) {
        print_usage(argv[0]);
        return DTO_USAGE;
    }

    const char *dtbo_path = argv[3];
    bool force = false;
    int timeout_ms = 2000;
    const char *check_node = NULL;

    // parse optional args
    for (int i = 4; i < argc; i++) {
        if (strcmp(argv[i], "--force") == 0) {
            force = true;
        } else if (strcmp(argv[i], "--timeout-ms") == 0) {
            if (i + 1 >= argc) { print_usage(argv[0]); return DTO_USAGE; }
            timeout_ms = atoi(argv[++i]);
            if (timeout_ms < 0) timeout_ms = 0;
        } else if (strcmp(argv[i], "--check-node") == 0) {
            if (i + 1 >= argc) { print_usage(argv[0]); return DTO_USAGE; }
            check_node = argv[++i];
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return DTO_USAGE;
        }
    }

    if (!path_exists(dtbo_path)) {
        fprintf(stderr, "Error: dtbo file not found: %s\n", dtbo_path);
        return DTO_USAGE;
    }

    // force remove if exists
    if (force) {
        if (rm_rf_overlay_dir(overlay_name) != 0) {
            perror("Error: --force unload existing overlay failed");
            return DTO_IO_FAIL;
        }
    }

    if (write_dtbo(overlay_name, dtbo_path) != 0) {
        // Wichtig: errno zeigt oft den echten Grund (EINVAL, ENOENT, ...)
        perror("Error: overlay load failed (writing dtbo)");
        fprintf(stderr,
            "Hint: If errno=EINVAL -> overlay invalid/doesn't match base DT or missing dtc -@\n"
            "      If errno=ENOENT -> target/label not found in base DT\n");
        return DTO_IO_FAIL;
    }

    if (wait_for_applied(overlay_name, timeout_ms) != 0) {
        perror("Error: overlay status did not become 'applied' in time");
        return DTO_APPLY_TIMEOUT;
    }

    if (check_node) {
        if (verify_proc_dt_node(check_node) != 0) {
            perror("Error: /proc/device-tree node check failed");
            fprintf(stderr, "Checked node path: %s\n", check_node);
            return DTO_VERIFY_FAIL;
        }
    }

    printf("OK: overlay '%s' applied.\n", overlay_name);
    if (check_node) printf("OK: node exists: /proc/device-tree/%s\n", check_node);
    return DTO_OK;
}
