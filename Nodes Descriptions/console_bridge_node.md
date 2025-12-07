# Console Bridge Node (`console_bridge_node.py`)

Subscribes to `/rosout`, re-formats each log entry with a human-friendly level
string and timestamp, then republishes it as plain text on
`console_bridge/log` using transient-local QoS for downstream dashboards.【F:src/auv_pkg/auv_pkg/console_bridge_node.py†L18-L57】

## Behaviour
- Creates a transient-local `String` publisher so late-joining tools can replay
  recent messages, and uses a reliable `/rosout` subscription to avoid drops
  when the system is busy.【F:src/auv_pkg/auv_pkg/console_bridge_node.py†L32-L48】
- Converts numeric ROS log levels into labels, embeds the floating-point
  timestamp, and trims trailing whitespace before publishing.【F:src/auv_pkg/auv_pkg/console_bridge_node.py†L18-L57】

## Interactions
- Intended as the first node launched so that subsequent nodes' logs are
  captured immediately and mirrored onto the plain-text topic for monitoring
  UIs or telemetry links.【F:src/auv_pkg/auv_pkg/console_bridge_node.py†L27-L64】
