"""
@ 2022 Carnegie Mellon University. All rights reserved.
National Robotics Engineering Center, Carnegie Mellon University
www.nrec.ri.cmu.edu
Confidential and Proprietary - Do not distribute without prior written
permission.

License Status: Not Released.
(License Status to be confirmed by CTTEC prior to release from NREC)
This notice must appear in all copies of this file and its derivatives.
"""

"""
NREC Internal Use (Use as Background IP to be cleared by CTTEC and the Project
Manager/PI prior to use on another project).
Created for Program: HPSTA - 55435.1.1990813
"""

import invariant_base
import signal
import rospy


class CrashInvariant(invariant_base.BaseInvariant):
    """Checks the SUT has crashed and report a violation when it does.

    It is an invariant that runs by default and has only one instance
    of itself. hence there is no name provided as an input.

    When SIGINT is received, the invariant interprets this as a crash
    from the SUT then it reports a violation.

    When SIGTERM is received, the invariant interprets this as everything
    has run properly and it initiates shutdown sequence for the code.
    """

    def __init__(self):  # noqa: D107
        super(CrashInvariant, self).__init__()

    def handle_sigint(self, sig, frame):
        """The signal sent to the crash invariant when the SUT crashes.
            It reports a violation when that happens and exits.

        Args:
            sig (int): signal numner requied for a signal handler
            frame (frame object): the current stack frame required for a signal handler

        Returns:
          None
        """
        self.report_violation()


def main():

    crash_invariant = CrashInvariant()
    signal.signal(signal.SIGINT, crash_invariant.handle_sigint)

    rospy.spin()


if __name__ == "__main__":
    main()
