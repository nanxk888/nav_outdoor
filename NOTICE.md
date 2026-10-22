# Third-party code and data

This repository has mixed licenses, not one blanket license:

- `src/d1h_fast_lio`: navigation-only derivative of FAST-LIVO2. GPLv2 text is
  retained in `LICENSE`, upstream attribution and modifications in `UPSTREAM.md`,
  and bundled headers retain their individual notices. Do not infer the license
  from an unrelated workspace's old package metadata.
- `src/d1h_nav2` and `src/d1h_global_localization`: package metadata declares
  Apache-2.0. External Nav2/robot_localization and dependency packages retain their
  own licenses; they are not copied into this repository.
- `map/xd_map.osm`: © OpenStreetMap contributors, available under the Open
  Database License (ODbL), https://www.openstreetmap.org/copyright . Retain
  attribution when displaying or distributing the map and derived data.
- Livox driver/SDK, vikit, Sophus and Orin device firmware are external
  dependencies. Their notices and distribution terms remain applicable.

No robot model weights, credentials, private GNSS correction-service accounts,
recorded camera data or real-world point-cloud maps are included.
