# Legacy auto format

These skeleton/variant files were the old way an Auto was stored: a shared skeleton of
commands plus a per-variant list of overrides. They have been migrated into `autos/`,
where each Auto is one self-contained file, and nothing reads them any more.

Kept only so a bad migration can be checked against the original. Safe to delete once
the autos in `autos/` look right.
