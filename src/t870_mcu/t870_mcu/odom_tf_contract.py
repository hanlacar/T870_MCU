"""Helpers that keep Odometry and odom->base_link TF exactly aligned."""


def copy_odom_pose_to_transform(odom, transform):
    """Copy frame, timestamp, position and quaternion without recomputation."""
    transform.header.stamp = odom.header.stamp
    transform.header.frame_id = odom.header.frame_id
    transform.child_frame_id = odom.child_frame_id
    transform.transform.translation.x = odom.pose.pose.position.x
    transform.transform.translation.y = odom.pose.pose.position.y
    transform.transform.translation.z = odom.pose.pose.position.z
    transform.transform.rotation.x = odom.pose.pose.orientation.x
    transform.transform.rotation.y = odom.pose.pose.orientation.y
    transform.transform.rotation.z = odom.pose.pose.orientation.z
    transform.transform.rotation.w = odom.pose.pose.orientation.w
    return transform


def validate_frame_contract(odom_frame, base_frame):
    """Return a stable diagnostic reason for invalid frame identifiers."""
    odom_frame = str(odom_frame).strip()
    base_frame = str(base_frame).strip()
    if not odom_frame or not base_frame:
        return False, "frame_id_empty"
    if odom_frame.startswith("/") or base_frame.startswith("/"):
        return False, "frame_id_must_not_start_with_slash"
    if odom_frame == base_frame:
        return False, "parent_equals_child"
    return True, "ok"


def odom_ownership_fault(other_odom_publishers, transform_exists,
                         base_frame="base_link"):
    """Return the fail-closed startup diagnostic for an existing owner."""
    owners = sorted(str(owner) for owner in other_odom_publishers)
    if owners or bool(transform_exists):
        return "FAIL_DUPLICATE_TF: existing odom->%s or /odom owner %s" % (
            base_frame, owners)
    return None


def odom_transform_matches(odom, transform, tolerance=1e-12):
    """Verify the wire-level pose contract, including the exact timestamp."""
    if (odom.header.frame_id != transform.header.frame_id or
            odom.child_frame_id != transform.child_frame_id):
        return False, "frame_id_mismatch"
    if (odom.header.stamp.sec != transform.header.stamp.sec or
            odom.header.stamp.nanosec != transform.header.stamp.nanosec):
        return False, "timestamp_mismatch"
    left = (
        odom.pose.pose.position.x, odom.pose.pose.position.y,
        odom.pose.pose.position.z, odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w)
    right = (
        transform.transform.translation.x, transform.transform.translation.y,
        transform.transform.translation.z, transform.transform.rotation.x,
        transform.transform.rotation.y, transform.transform.rotation.z,
        transform.transform.rotation.w)
    if any(abs(float(a) - float(b)) > tolerance for a, b in zip(left, right)):
        return False, "pose_mismatch"
    return True, "ok"
