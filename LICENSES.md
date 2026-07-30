# Licensing

## Files tracked directly by this repository

RoboParty-owned source code and other copyrightable material tracked directly
by this repository are licensed under the
[GNU General Public License version 3 only](./LICENSE), SPDX identifier
`GPL-3.0-only`, unless an individual file states otherwise.

This statement applies only to material that RoboParty owns or has authority to
license. It does not override notices already present in individual files.

## Git submodules

`robots_localization_ros2` and `nlink_parser_ros2` are independent Git
submodules. The parent repository's GPL-3.0-only licence does not automatically
relicense their contents.

At the time of the licensing review:

- `robots_localization_ros2/package.xml` declared `<license>TODO</license>`
  and no root licence was identified; and
- no root licence was identified for the referenced
  `nlink_parser_ros2` repository.

Do not assume that these submodules are GPL-3.0-only merely because they are
referenced here. Confirm their upstream terms before redistributing their
source, including in a bundled archive or release.

## External dependencies

ROS 2, Nav2, FAST-LIO, ikd-Tree, Livox SDK/driver, Sophus, fmt, Open3D and other
external dependencies remain under their respective licences. Installation
instructions and acknowledgements do not relicense those projects.
