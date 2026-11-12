"""Cleanup ordering independent of ROS, including a disappearing terminal."""
import termios


def cleanup_session(stop, restore_terminal, destroy_node, shutdown_ros):
    """Attempt stopping before terminal operations; always release ROS resources.

    A terminal hangup commonly makes tcsetattr raise EIO. Terminal restoration
    must neither precede the stop attempt nor prevent node/context cleanup.
    The caller uses TCSANOW, because waiting for an abandoned TTY to drain can
    block. Stop transport failures remain visible after cleanup.
    """
    try:
        stop()
    finally:
        try:
            try:
                restore_terminal()
            except (termios.error, OSError):
                pass
        finally:
            try:
                destroy_node()
            finally:
                shutdown_ros()
