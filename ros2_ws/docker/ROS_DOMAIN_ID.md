# ROS_DOMAIN_ID — Per-Team Setup

## Why this matters

ROS2 uses DDS (Data Distribution Service) for node discovery. By default all ROS2
nodes on the same network join **domain 0**, which means nodes on every team's
Raspberry Pi are visible to every other team. This causes:

- `ros2 node list` showing 5–10 bridge or robot nodes when only one should exist
- The NUEVO UI showing "view only" because conflicting publishers confuse the bridge
- Unpredictable behaviour when multiple teams' robots respond to the same topics

Setting a unique `ROS_DOMAIN_ID` per team creates an isolated namespace. Teams on
different domain IDs are completely invisible to each other.

## How to set it

Open `ros2_ws/docker/docker-compose.rpi.yml` in your fork and change the sentinel
value `999` to your group number:

```yaml
environment:
  - ROS_DOMAIN_ID=24    # section 2, group 4 → 24
```

Valid values are **0 to 101**. Use your group number to keep it easy to remember.

Then restart the container for the change to take effect:

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
docker compose -f $COMPOSE down
docker compose -f $COMPOSE up -d
docker compose -f $COMPOSE logs -f ros2_runtime
```

## Verifying it works

After startup, exec into the container and confirm only your own nodes appear:

```bash
docker compose -f $COMPOSE exec ros2_runtime bash -c \
  "source /ros2_ws/install/setup.bash && ros2 node list"
```

You should see only `/bridge` (after launching the bridge node). If you see
duplicate nodes from other teams, your `ROS_DOMAIN_ID` has not been updated or
the container has not been restarted.

## What happens if you forget

The container will refuse to start normally. Instead it prints a clear error in
the logs and idles (`sleep infinity`) so the message stays visible:

```
════════════════════════════════════════════════════════════════
  ERROR: ROS_DOMAIN_ID is not configured for this team.

  Edit ros2_ws/docker/docker-compose.rpi.yml and set:
    - ROS_DOMAIN_ID=<your-group-number>    # must be 0–101
  ...
════════════════════════════════════════════════════════════════
```

## Domain ID assignments

The domain ID is your **section number followed by your group number**:
`ROS_DOMAIN_ID = <section><group>`

| Section | Group | ROS_DOMAIN_ID |
|---------|-------|--------------|
| 2       | 1     | 21           |
| 2       | 2     | 22           |
| 2       | 3     | 23           |
| 2       | 4     | 24           |
| 2       | 5     | 25           |
| 2       | 6     | 26           |
| 2       | 7     | 27           |
| 2       | 8     | 28           |
| 4       | 1     | 41           |
| 4       | 2     | 42           |
| 4       | 3     | 43           |
| 4       | 4     | 44           |
| 4       | 5     | 45           |
| 4       | 6     | 46           |
| 4       | 7     | 47           |
| 4       | 8     | 48           |
