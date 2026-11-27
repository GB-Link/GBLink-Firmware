#!/usr/bin/env bash
# Install udev rules so any local user can open the GBLink adapter via
# WebUSB or WebSerial without sudo. Covers the Zephyr firmware
# (1209:B011 from 3.0.0, VID 0x2FE3 before), the reconfigurable firmware
# (VID 0xCAFE), and the RP2040/RP2350 bootrom (VID 0x2E8A) used when
# flashing in BOOTSEL mode,
# and tells ModemManager not to probe the CDC-ACM port.
#
# After installing the rules, if any flatpak browsers are detected, offers
# to grant them `--device=all` so the sandbox can see the adapter. On
# uninstall, offers to revoke that grant.

set -euo pipefail

RULES_PATH=/etc/udev/rules.d/50-gblink-adapter.rules

# Browsers that may need WebUSB/WebSerial access from inside a flatpak sandbox.
FLATPAK_BROWSERS=(
    org.mozilla.firefox
    io.gitlab.librewolf-community
    app.zen_browser.zen
    com.brave.Browser
    org.chromium.Chromium
    com.google.Chrome
    com.microsoft.Edge
    com.vivaldi.Vivaldi
    net.mullvad.MullvadBrowser
)

ACTION=install

# When invoked by the sudo re-exec, --action carries the user's earlier
# choice — skip the prompt so we don't ask twice.
if [[ "${1:-}" == "--action" && -n "${2:-}" ]]; then
    ACTION="$2"
elif [[ -e "$RULES_PATH" ]]; then
    echo "GBLink udev rules already installed at $RULES_PATH."
    read -rp "[R]einstall, [U]ninstall, [C]ancel? " choice
    case "${choice,,}" in
        r|reinstall) ACTION=install ;;
        u|uninstall) ACTION=uninstall ;;
        *)           echo "Cancelled."; exit 0 ;;
    esac
fi

if [[ $EUID -ne 0 ]]; then
    exec sudo -E -- "$0" --action "$ACTION"
fi

# --- Flatpak browser helpers (run after udev work, as the invoking user) ---

# Run a command as the original (non-root) user. `flatpak override --user`
# writes to that user's $HOME and must not run as root.
run_as_user() {
    if [[ -n "${SUDO_USER:-}" ]]; then
        sudo -u "$SUDO_USER" -- "$@"
    else
        "$@"
    fi
}

# Echo the user's $HOME, even when we're running as root via sudo.
user_home() {
    if [[ -n "${SUDO_USER:-}" ]]; then
        getent passwd "$SUDO_USER" | cut -d: -f6
    else
        printf '%s\n' "$HOME"
    fi
}

# Echo each installed flatpak browser app-ID, one per line.
installed_flatpak_browsers() {
    command -v flatpak >/dev/null 2>&1 || return
    local installed app
    installed=$(run_as_user flatpak list --app --columns=application 2>/dev/null || true)
    [[ -z "$installed" ]] && return
    for app in "${FLATPAK_BROWSERS[@]}"; do
        grep -qxF "$app" <<<"$installed" && printf '%s\n' "$app"
    done
}

# Echo each detected browser that has either of our overrides currently set
# (devices=all for the device nodes, filesystems=/run/udev:ro for the udev
# hot-plug socket Firefox flatpak needs to enumerate serial ports).
browsers_with_device_override() {
    local home app override_file
    home=$(user_home)
    while read -r app; do
        override_file="$home/.local/share/flatpak/overrides/$app"
        [[ -f "$override_file" ]] || continue
        if grep -Eq '^devices=all;?$' "$override_file" \
            || grep -Eq '^filesystems=.*(^|;)/run/udev:ro(;|$)' "$override_file"; then
            printf '%s\n' "$app"
        fi
    done < <(installed_flatpak_browsers)
}

# Surgically strip our two grants from a flatpak override file, preserving
# anything else the user set. Deletes the file if nothing meaningful remains.
# Safer than `flatpak override --user --reset`, which would also nuke
# unrelated overrides.
remove_device_all_override() {
    local app="$1" home f
    home=$(user_home)
    f="$home/.local/share/flatpak/overrides/$app"
    [[ -f "$f" ]] || return
    run_as_user sed -i -E \
        -e '/^devices=all;?$/d' \
        -e '/^filesystems=/ s|/run/udev:ro;||' \
        -e '/^filesystems=$/d' \
        "$f"
    # If only the [Context] header (or whitespace) remains, drop the file.
    if [[ -z "$(grep -vE '^(\[Context\]|[[:space:]]*)$' "$f" 2>/dev/null || true)" ]]; then
        run_as_user rm -f "$f"
    fi
}

# --- Uninstall flow ---

if [[ "$ACTION" == "uninstall" ]]; then
    rm -f "$RULES_PATH"
    udevadm control --reload-rules
    udevadm trigger
    echo "Removed $RULES_PATH."

    overrides=$(browsers_with_device_override || true)
    if [[ -n "$overrides" ]]; then
        echo
        echo "These flatpak browsers currently have device-access overrides:"
        printf '  - %s\n' $overrides
        read -rp "Revoke device access for them? [y/N] " choice
        case "${choice,,}" in
            y|yes)
                while read -r app; do
                    remove_device_all_override "$app"
                    echo "  revoked: $app"
                done <<<"$overrides"
                ;;
            *) echo "Left flatpak overrides untouched." ;;
        esac
    fi

    echo
    echo "Replug the adapter."
    exit 0
fi

# --- Install flow ---

cat >"$RULES_PATH" <<'EOF'
# GBLink adapter — WebUSB (vendor interface) + WebSerial (CDC-ACM).
# Covers Zephyr firmware (1209:B011 from 3.0.0, VID 0x2FE3 before) and
# reconfigurable firmware (VID 0xCAFE). 0x1209 is the pid.codes vendor ID, shared
# by many projects, so it is matched together with the product ID.
# Also covers the RP2040/RP2350 bootrom (VID 0x2E8A) in BOOTSEL mode, so the
# board can be flashed over WebUSB (e.g. picoflash) without sudo.

SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="b011", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="2fe3", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="cafe", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", MODE="0666"

SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="b011", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2fe3", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="cafe", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF

chmod 0644 "$RULES_PATH"
udevadm control --reload-rules
udevadm trigger

echo
echo "Installed udev rules at $RULES_PATH."

# Offer to grant flatpak browsers device access (sandboxes hide /dev nodes
# from the host even after udev rules grant world-access).
browsers=$(installed_flatpak_browsers || true)
if [[ -n "$browsers" ]]; then
    echo
    echo "Detected flatpak browsers that need extra access for WebUSB/WebSerial:"
    printf '  - %s\n' $browsers
    read -rp "Grant them device + udev access so they can see the adapter? [Y/n] " choice
    case "${choice,,}" in
        n|no) echo "Skipped flatpak overrides." ;;
        *)
            while read -r app; do
                run_as_user flatpak override --user --device=all "$app"
                run_as_user flatpak override --user --filesystem=/run/udev:ro "$app"
                echo "  granted: $app"
            done <<<"$browsers"
            echo "Fully quit any open flatpak browsers (pkill firefox / brave / etc.)"
            echo "for the change to take effect."
            ;;
    esac
fi

echo
echo "Unplug and replug the GBLink adapter, then refresh your browser. The"
echo "\"Connect USB\" and \"Connect Serial\" buttons should work without sudo."
