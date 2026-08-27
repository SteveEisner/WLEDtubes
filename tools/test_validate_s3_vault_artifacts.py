import importlib.util
import pathlib
import struct
import sys
import tempfile
import unittest

MODULE_PATH = pathlib.Path(__file__).with_name("validate_s3_vault_artifacts.py")
SPEC = importlib.util.spec_from_file_location("validate_s3_vault_artifacts", MODULE_PATH)
validator = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = validator
SPEC.loader.exec_module(validator)


class ArtifactValidatorTest(unittest.TestCase):
    def fixture(self, family: int, release: int = 40, extra: bytes = b"") -> pathlib.Path:
        path = pathlib.Path(self.temporary.name) / f"{family}-{len(extra)}.bin"
        identity = struct.pack("<8sBBBH3s", validator.MAGIC, 1, family, 0, release, b"\0\0\0")
        path.write_bytes(b"firmware" + identity + extra)
        return path

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.temporary.cleanup()

    def test_accepts_exact_standard_profiles(self):
        self.assertEqual(validator.inspect("dig2go", self.fixture(1), 40).family, 1)
        self.assertEqual(validator.inspect("athom-c3", self.fixture(3), 40).family, 3)

    def test_rejects_wrong_family_release_and_duplicate_identity(self):
        with self.assertRaises(ValueError):
            validator.inspect("dig2go", self.fixture(3), 40)
        with self.assertRaises(ValueError):
            validator.inspect("dig2go", self.fixture(1, 39), 40)
        duplicate = struct.pack("<8sBBBH3s", validator.MAGIC, 1, 1, 0, 40, b"\0\0\0")
        with self.assertRaises(ValueError):
            validator.inspect("dig2go", self.fixture(1, extra=duplicate), 40)


if __name__ == "__main__":
    unittest.main()
