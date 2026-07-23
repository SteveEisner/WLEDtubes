import pathlib
import sys
import unittest


REPO_DIR = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_DIR / "usermods" / "Tubes"))
import mesh_device_report as REPORTS  # noqa: E402


VALID_REPORT = (
    "TUBE_REPORT nonce=89ABCDEF mac=5443b2b542f4 family=1 variant=0 tubes=14 "
    "release=092C041A leds=112 buses=1 pin=16 type=22 "
    "role=10 mesh=3 node=1234 uplink=3850 uptime=9"
)


class MeshDeviceReportTest(unittest.TestCase):
    def test_general_dig2go_profile_does_not_force_an_installation_mode(self) -> None:
        """Fleet firmware must respect stored roles instead of compiling every tube as master."""
        config = (REPO_DIR / "platformio_tubes.ini").read_text()
        profile = config.split("[env:esp32_quinled_dig2go_tubes]", 1)[1].split(
            "# ------------------------------------------------------------------------------", 1
        )[0]

        self.assertNotIn("-D CHRISTMAS", profile)
        self.assertNotIn("-D GOLDEN", profile)
        self.assertNotIn("-D RUBY", profile)

    def test_parses_machine_readable_firmware_report(self) -> None:
        """A serial report preserves every field needed for post-reboot verification."""
        report = REPORTS.parse_report_line(VALID_REPORT)

        self.assertIsNotNone(report)
        assert report is not None
        self.assertEqual(report.nonce, 0x89ABCDEF)
        self.assertEqual(report.mac, "5443b2b542f4")
        self.assertEqual(report.family, REPORTS.FAMILY_IDS["dig2go"])
        self.assertEqual(report.leds, 112)
        self.assertEqual(report.pin, 16)
        self.assertEqual(report.uplink, 3850)

    def test_parses_report_after_mesh_routing_trace(self) -> None:
        """The controller prints its route prefix before invoking the report handler."""
        report = REPORTS.parse_report_line(">> FFF/000 ACTION -48dB  " + VALID_REPORT)

        self.assertIsNotNone(report)
        assert report is not None
        self.assertEqual(report.mac, "5443b2b542f4")

    def test_rejects_incomplete_serial_report(self) -> None:
        """A truncated report cannot supply partial evidence for a successful upgrade."""
        report = REPORTS.parse_report_line("TUBE_REPORT nonce=89ABCDEF mac=5443b2b542f4")

        self.assertIsNone(report)

    def test_requires_firmware_family_and_preserved_led_configuration(self) -> None:
        """Verification succeeds only when identity, release, and active output all match."""
        expected_release = "DIG2GO_TUBES"
        release_hash = REPORTS.djb2(expected_release)
        line = VALID_REPORT.replace("092C041A", f"{release_hash:08X}")
        report = REPORTS.parse_report_line(line)
        assert report is not None

        matching = REPORTS.report_mismatch(
            report,
            "54:43:b2:b5:42:f4",
            REPORTS.FAMILY_IDS["dig2go"],
            0,
            expected_release,
            14,
            112,
            16,
            22,
        )
        wrong_count = REPORTS.report_mismatch(
            report,
            report.mac,
            report.family,
            report.variant,
            expected_release,
            report.tubes,
            300,
            report.pin,
            report.type,
        )

        self.assertIsNone(matching)
        self.assertEqual(wrong_count, "leds is 112, expected 300")

    def test_rejects_a_non_master_that_is_leading_or_forced_into_master_mode(self) -> None:
        """A valid OTA is not enough when the device is detached from mesh control."""
        expected_release = "DIG2GO_TUBES"
        valid_line = VALID_REPORT.replace("092C041A", f"{REPORTS.djb2(expected_release):08X}")
        leading = REPORTS.parse_report_line(valid_line.replace("mesh=3", "mesh=1").replace("uplink=3850", "uplink=0"))
        forced_master = REPORTS.parse_report_line(valid_line.replace("mesh=3", "mesh=7"))
        assert leading is not None
        assert forced_master is not None

        expected = (
            leading.mac,
            leading.family,
            leading.variant,
            expected_release,
            leading.tubes,
            leading.leds,
            leading.pin,
            leading.type,
        )
        self.assertEqual(
            REPORTS.report_mismatch(leading, *expected),
            "non-master device has not joined an uplink",
        )
        self.assertEqual(
            REPORTS.report_mismatch(forced_master, *expected),
            "master behavior is True, but stored role 10 expects False",
        )

    def test_special_variant_is_expected_to_run_as_master(self) -> None:
        """Installation variants intentionally retain their forced master behavior."""
        expected_release = "CHRISTMAS_TUBES"
        line = VALID_REPORT.replace("variant=0", "variant=1").replace(
            "092C041A", f"{REPORTS.djb2(expected_release):08X}"
        ).replace("mesh=3", "mesh=5").replace("node=1234", "node=4095").replace(
            "uplink=3850", "uplink=0"
        )
        report = REPORTS.parse_report_line(line)
        assert report is not None

        self.assertIsNone(
            REPORTS.report_mismatch(
                report,
                report.mac,
                report.family,
                1,
                expected_release,
                report.tubes,
                report.leds,
                report.pin,
                report.type,
            )
        )


if __name__ == "__main__":
    unittest.main()
