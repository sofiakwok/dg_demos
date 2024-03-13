import torch


class PDController(torch.nn.Module):
    def __init__(self):
        super(PDController, self).__init__()

    def forward(self, kp, kd, position, velocity, des_position, des_velocity):
        return kp * (des_position - position) + kd * (des_velocity - velocity)


pd_controller = PDController()
sm = torch.jit.script(pd_controller)

sm.save("script_pd_controller.pt")

# Run the module with some example data.
kp = torch.tensor([5.0])
kd = torch.tensor([0.1])
velocity = torch.zeros([12])
position = torch.zeros([12])
des_velocity = torch.ones([12])
des_position = torch.ones([12])

res = pd_controller.forward(
    kp, kd, position, velocity, des_position, des_velocity
)
print(res)
