# NOTE: taken from Humanoid-MuJoCo repo! If any changes are made there, they must be reflected here!
import torch
import torch.nn as nn
import numpy as np

device = torch.device('cuda:0') if torch.cuda.is_available() else torch.device('cpu')

class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(ActorCritic, self).__init__()

        # actor
        self.actor = nn.Sequential(
                        nn.Linear(state_dim, 64),
                        nn.Tanh(),
                        nn.Linear(64, 64),
                        nn.Tanh(),
                        nn.Linear(64, action_dim),
                        nn.Tanh()
                    )
        # critic
        self.critic = nn.Sequential(
                        nn.Linear(state_dim, 64),
                        nn.Tanh(),
                        nn.Linear(64, 64),
                        nn.Tanh(),
                        nn.Linear(64, 1)
                    )
    
    def act(self, state):
        action = self.actor(state)
        return action.detach().cpu().numpy()
    
    def load(self, checkpoint_path):
        self.load_state_dict(torch.load(checkpoint_path, map_location=lambda storage, loc: storage))