import os
import pty
import termios
import unittest
from control_cleanup import cleanup_session


class CleanupTests(unittest.TestCase):
    def test_real_pty_hangup_restore_does_not_skip_stop_or_release(self):
        master,slave=pty.openpty()
        saved=termios.tcgetattr(slave)
        os.close(master)
        order=[]
        def restore():
            order.append('restore')
            # Closing the master causes this restoration to fail on Linux/macOS.
            termios.tcsetattr(slave,termios.TCSANOW,saved)
        try:
            cleanup_session(lambda:order.append('stop'),restore,
                            lambda:order.append('destroy'),lambda:order.append('shutdown'))
        finally: os.close(slave)
        self.assertEqual(order,['stop','restore','destroy','shutdown'])

    def test_explicit_termios_error_cannot_skip_cleanup(self):
        order=[]
        def restore():
            order.append('restore');raise termios.error(5,'Input/output error')
        cleanup_session(lambda:order.append('stop'),restore,
                        lambda:order.append('destroy'),lambda:order.append('shutdown'))
        self.assertEqual(order,['stop','restore','destroy','shutdown'])

    def test_transport_stop_error_still_releases_everything(self):
        order=[]
        def stop():
            order.append('stop');raise RuntimeError('transport failed')
        with self.assertRaises(RuntimeError):
            cleanup_session(stop,lambda:order.append('restore'),
                            lambda:order.append('destroy'),lambda:order.append('shutdown'))
        self.assertEqual(order,['stop','restore','destroy','shutdown'])

    def test_node_destroy_error_still_shuts_down_context(self):
        order=[]
        def destroy():
            order.append('destroy');raise RuntimeError('destroy failed')
        with self.assertRaises(RuntimeError):
            cleanup_session(lambda:order.append('stop'),lambda:order.append('restore'),
                            destroy,lambda:order.append('shutdown'))
        self.assertEqual(order,['stop','restore','destroy','shutdown'])


if __name__=='__main__':unittest.main()
