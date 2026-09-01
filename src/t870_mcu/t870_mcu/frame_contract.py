"""Static vehicle TF configuration validation."""


def validate_enabled_tree(frames, root="base_link"):
    """Detect orphaned parents, cycles, and attempts to statically own root."""
    enabled = {str(child): cfg for child, cfg in (frames or {}).items()
               if bool((cfg or {}).get("enabled", True))}
    errors = []
    if root in enabled:
        errors.append("root_must_not_be_static_child:%s" % root)
    for child, cfg in enabled.items():
        parent = str((cfg or {}).get("parent", root))
        if not parent or parent == child:
            errors.append("invalid_parent:%s" % child)
            continue
        seen = {child}
        current = parent
        while current != root:
            if current in seen:
                errors.append("cycle:%s" % child)
                break
            seen.add(current)
            if current not in enabled:
                errors.append("orphan:%s->%s" % (child, current))
                break
            current = str(enabled[current].get("parent", root))
    return sorted(set(errors))
