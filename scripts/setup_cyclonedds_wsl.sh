#!/usr/bin/env bash
set -e

PI_IP="${1:-192.168.50.124}"

mkdir -p ~/.config/cyclonedds
cat > ~/.config/cyclonedds/cyclonedds.xml <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="127.0.0.1"/>
        <Peer address="${PI_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "export CYCLONEDDS_URI=file://$HOME/.config/cyclonedds/cyclonedds.xml"
