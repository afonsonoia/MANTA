#!/usr/bin/env python3
"""Tests for MANTA_PI boot manager, service configuration, and Wi-Fi lockout prevention.

Ensures that:
1. manta-boot.service utilizes privileged ExecStartPre (+) for hardware unblock.
2. manta_boot.sh implements multi-layer Wi-Fi reactivation (rfkill, nmcli, ip link, rescan).
3. Connection detection is robust and avoids false negatives from ICMP drops.
4. All shell scripts in Code/MANTA_PI maintain strict Unix LF line endings.
"""

import os
import re
import unittest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MANTA_PI_DIR = os.path.join(REPO_ROOT, "Code", "MANTA_PI")
BOOT_SCRIPT = os.path.join(MANTA_PI_DIR, "manta_boot.sh")
BOOT_SERVICE = os.path.join(MANTA_PI_DIR, "manta-boot.service")


class TestMantaPiBoot(unittest.TestCase):
    """Test suite for MANTA Raspberry Pi boot management."""

    def test_boot_service_privileged_unblock(self):
        """Verify manta-boot.service uses ExecStartPre with root privilege (+) to unblock radios."""
        self.assertTrue(os.path.isfile(BOOT_SERVICE), f"Missing {BOOT_SERVICE}")
        with open(BOOT_SERVICE, "r", encoding="utf-8") as f:
            content = f.read()

        # Must execute rfkill unblock with '+' prefix (root override)
        self.assertIn("ExecStartPre=+", content, "manta-boot.service must contain ExecStartPre=+ for root execution")
        self.assertIn("rfkill unblock all", content, "ExecStartPre must unblock all rfkill radios")
        self.assertIn("nmcli radio wifi on", content, "ExecStartPre must re-enable NetworkManager Wi-Fi")
        self.assertIn("ExecStart=/bin/bash /home/pc/MANTA/Code/MANTA_PI/manta_boot.sh", content)
        self.assertIn("User=pc", content)

    def test_boot_script_multi_layer_reactivation(self):
        """Verify manta_boot.sh unblocks rfkill, NetworkManager, and sets interfaces UP on boot."""
        self.assertTrue(os.path.isfile(BOOT_SCRIPT), f"Missing {BOOT_SCRIPT}")
        with open(BOOT_SCRIPT, "r", encoding="utf-8") as f:
            content = f.read()

        # Kernel/Driver rfkill layer
        self.assertIn("rfkill unblock wifi", content)
        self.assertIn("rfkill unblock all", content)

        # NetworkManager layer
        self.assertIn("nmcli radio wifi on", content)
        self.assertIn("nmcli networking on", content)

        # Interface UP layer
        self.assertTrue(
            re.search(r"ip link set [\"']?\$iface[\"']? up", content),
            "manta_boot.sh must explicitly bring wireless interfaces UP",
        )

        # Active rescan
        self.assertTrue(
            "device wifi rescan" in content or "reassociate" in content,
            "manta_boot.sh must trigger active Wi-Fi scan/reassociation",
        )

    def test_boot_script_connection_detection(self):
        """Verify manta_boot.sh implements intelligent connection check with IP, nmcli, and ping."""
        with open(BOOT_SCRIPT, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("check_wifi_connection", content)
        # NetworkManager connected check
        self.assertIn("wifi:connected", content)
        # IPv4 assigned check
        self.assertTrue(
            re.search(r"ip -4 -o addr show", content),
            "manta_boot.sh must verify assigned IPv4 on wireless interface",
        )
        # Ping fallback check
        self.assertIn("GATEWAY", content)
        self.assertIn("ping -c 1", content)

    def test_script_unix_line_endings(self):
        """Verify all scripts and service units use strict LF (no Windows CRLF \r\n)."""
        target_files = [
            os.path.join(MANTA_PI_DIR, f)
            for f in os.listdir(MANTA_PI_DIR)
            if f.endswith((".sh", ".service"))
        ]
        self.assertGreater(len(target_files), 0, "Should find .sh and .service files in MANTA_PI")

        for filepath in target_files:
            with open(filepath, "rb") as f:
                raw = f.read()
            self.assertNotIn(
                b"\r\n",
                raw,
                f"File {os.path.basename(filepath)} contains Windows CRLF line endings, which break bash on Linux.",
            )


if __name__ == "__main__":
    unittest.main()
