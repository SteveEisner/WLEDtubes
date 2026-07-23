import importlib.util
import pathlib
import struct
import tempfile
import unittest
from typing import Optional


REPO_DIR = pathlib.Path(__file__).resolve().parents[2]
MODULE_PATH = REPO_DIR / "usermods" / "Tubes" / "read_firmware_metadata.py"
SPEC = importlib.util.spec_from_file_location("read_firmware_metadata", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
METADATA = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(METADATA)


def firmware_record(release: str, release_hash: Optional[int] = None) -> bytes:
    encoded_release = release.encode("ascii")
    encoded_version = b"0.15.3"
    if release_hash is None:
        release_hash = METADATA.djb2(encoded_release)
    record = METADATA.WLED_METADATA.pack(
        METADATA.WLED_METADATA_MAGIC,
        2,
        encoded_version.ljust(48, b"\0"),
        encoded_release.ljust(48, b"\0"),
        release_hash,
        0,
        0,
        0,
    )
    return b"firmware-prefix" + record + b"firmware-suffix"


class FirmwareMetadataTest(unittest.TestCase):
    def read_fixture(self, contents: bytes) -> tuple[str, str]:
        with tempfile.TemporaryDirectory() as temporary_dir:
            firmware_path = pathlib.Path(temporary_dir) / "firmware.bin"
            firmware_path.write_bytes(contents)
            return METADATA.read_firmware_metadata(firmware_path)

    def test_returns_valid_hardware_family(self) -> None:
        """A valid WLED record exposes the version and release used for upload authorization."""
        identity = self.read_fixture(firmware_record("DIG2GO_TUBES"))

        self.assertEqual(identity, ("0.15.3", "DIG2GO_TUBES"))

    def test_rejects_release_with_corrupt_hash(self) -> None:
        """A release name whose WLED hash is corrupt cannot authorize a hardware upload."""
        with self.assertRaisesRegex(ValueError, "no valid WLED compatibility metadata"):
            self.read_fixture(firmware_record("DIG2GO_TUBES", release_hash=1))

    def test_rejects_image_without_metadata(self) -> None:
        """A binary without WLED compatibility metadata is rejected before device contact."""
        with self.assertRaisesRegex(ValueError, "no valid WLED compatibility metadata"):
            self.read_fixture(b"not-a-wled-image")


if __name__ == "__main__":
    unittest.main()
