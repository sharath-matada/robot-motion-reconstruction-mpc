import robotoc_sim


class A1Simulator(robotoc_sim.LeggedSimulator):
    def __init__(self, urdf_path, time_step):
        super().__init__(urdf_path, time_step)

    @classmethod
    def get_joint_id_map(self):
        return [9, 10, 11, 2, 3, 4, 23, 24, 25, 16, 17, 18]