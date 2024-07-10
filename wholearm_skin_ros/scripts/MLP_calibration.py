#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pickle5 as pickle
import torch
import tqdm
from sklearn.model_selection import train_test_split
from torch.utils.data import Dataset, DataLoader, TensorDataset
import os
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from itertools import chain
import copy
import wandb

# Create custom data set
class SkinDataSet(Dataset):
    def __init__(self, directory):
        self.data = []
        self.labels = []
        self.X_mean_list = []
        self.X_std_list = []
        self.X_scaled = []
        self.y_mean_list = []
        self.y_std_list = []
        self.y_scaled = []
        self.load_data(directory)
    
    def load_data(self, directory):
        for filename in os.listdir(directory):
            if filename.endswith(".pickle"):
                file_path = os.path.join(directory, filename)
                with open(file_path, "rb") as f:
                    pickle_data = pickle.load(f)
                    self.process_data(pickle_data)

        # Scale the feature and label data using (X - mean) / std
        for col in self.data:
            scale(col, self.X_mean_list, self.X_std_list)

        for col in self.labels:
            scale(col, self.y_mean_list, self.y_std_list)

    def process_data(self, data):
        ft = np.array(data['ft'])
        skin = np.array(data['skin'])
        joint_pos = np.array(data['joint_pos'])
        joint_vel = np.array(data['joint_vel'])
        joint_effort = np.array(data['joint_effort'])
        taxel_pos = np.array(data['taxel_pos'])
        taxel_ori= np.array(data['taxel_ori'])
        taxel = np.array(data['taxel'])

        # Reformatting data
        joint_pos = [linklist[1:] for linklist in joint_pos]
        joint_vel = [linklist[1:] for linklist in joint_vel]
        joint_effort = [linklist[1:] for linklist in joint_effort]

        joint_pos = separate_cols(joint_pos)
        joint_vel = separate_cols(joint_vel)
        joint_effort = separate_cols(joint_effort)

        taxel_pos = Position.extract_pos(taxel_pos)
        taxel_pos = separate_cols(taxel_pos)
        taxel_ori = Orientation.extract_ori(taxel_ori)
        taxel_ori = separate_cols(taxel_ori)

        # Combine features into a single list
        X = np.vstack((skin, joint_pos, joint_vel, joint_effort, taxel_pos, taxel_ori))
        X = separate_cols(X)
        y = ft

        if(len(self.data) == 0):
            self.data = X
            self.labels = y
        else:
            self.data = np.concatenate((self.data, np.array(X)))
            self.labels = np.concatenate((self.labels, np.array(y)))
        # print("data shape", np.array(self.data).shape)

    def __len__(self):
        # return 100
        return len(self.data)

    def __getitem__(self, index):
        features = torch.tensor(self.data[index], dtype=torch.float32)#.cuda()
        label = torch.tensor(self.labels[index], dtype=torch.float32)#.cuda()
        return features, label

class Position:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        
    def extract_pos(pos):
        """
        Extracts the values for x, y, and z in the position object
        """
        float_list = []
        for obj in pos:
            float_list.append([obj.x, obj.y, obj.z])
        return float_list

class Orientation:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        
    def extract_ori(ori):
        """
        Extracts the values for x, y, and z in the position object
        """
        float_list = []
        for obj in ori:
            float_list.append([obj.x, obj.y, obj.z, obj.w])
        return float_list

def flatten(nested_list):
        """
        Flatten out a level of nested lists
        """
        return list(chain.from_iterable(nested_list))

def separate_cols(nested_list):
    """
    Separates nested list into the columns of each element in the list
    """
    # cols = list(zip(*nested_list))
    cols = list(map(list, zip(*nested_list)))
    return cols

def scale(data, mean_list, std_list):
        """
        Calculates the mean and standard deviation of the data and adds them to a list.
        Scales the data using z = (x - mean) / std
        """
        mean = np.sum(data)/len(data)
        std = np.std(data)
        mean_list.append(mean)
        std_list.append(std)
        for i in range(len(data)):
            if(std == 0):
                data[i] = 0
            else:
                data[i] = (data[i] - mean) / std

        # min = np.min(data)
        # max = np.max(data)
        # for i in range(len(data)):
        #     data[i] = 2 * (data[i] - min) / (max - min) - 1

# create MLP
class MLP(nn.Module):
    def __init__(self, input_size, hidden_size, output_size, dropout):
        super(MLP, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.relu1 = nn.ReLU()
        self.dropout1 = nn.Dropout(dropout)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.relu2 = nn.ReLU()
        self.dropout2 = nn.Dropout(dropout)
        self.fc3 = nn.Linear(hidden_size, hidden_size)
        self.relu3 = nn.ReLU()
        self.dropout3 = nn.Dropout(dropout)
        self.fc4 = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        x = self.fc1(x) # First hidden layer
        x = self.relu1(x)
        x = self.dropout1(x) # Dropout layer
        x = self.fc2(x) # Second hidden layer
        x = self.relu2(x)
        x = self.dropout2(x)
        x = self.fc3(x) # Third hidden layer
        x = self.relu3(x)
        x = self.dropout3(x)
        x = self.fc4(x) # Output layer
        return x

class RMSELoss(nn.Module):
    def __init__(self):
        super(RMSELoss, self).__init__()
        self.mse = nn.MSELoss()

    def forward(self, y_pred, y_true):
        return torch.sqrt(self.mse(y_pred, y_true))

wandb.login()

# train model
def train(config=None):
    with wandb.init() as run:
        config = wandb.config

        # TODO: Add train and test pickle files inside their respsective folders
        train_dataset = SkinDataSet('/home/emprise/wholearm_ws/wholearm_ws/MLP_train')
        test_dataset = SkinDataSet('/home/emprise/wholearm_ws/wholearm_ws/MLP_test')

        # train_loader = DataLoader(train_dataset, batch_size=config.batch_size, shuffle=True)
        # test_loader = DataLoader(test_dataset, batch_size=config.batch_size, shuffle=False)
        train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
        test_loader = DataLoader(test_dataset, batch_size=128, shuffle=False)

        # create model
        model = MLP(len(train_dataset.data[0]), config.hidden_size, 3, config.dropout)#.cuda()

        # define loss function and optimizer
        criterion = RMSELoss()
        optimizer = optim.Adam(model.parameters(), lr=config.lr, weight_decay=1e-5) # weight_decay=1e-5

        best_weights = copy.deepcopy(model.state_dict())
        best_loss = np.inf
        train_losses = []
        test_losses = []

        for epoch in range(config.epochs):
            model.train()
            running_train_loss = 0.0
            for i, (input_batch, target_batch) in enumerate(train_loader):
                outputs = model(input_batch)
                loss = criterion(outputs, target_batch)
                
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                running_train_loss += loss.item() * input_batch.size(0)
            
            epoch_train_loss = running_train_loss / len(train_loader.dataset)
            train_losses.append(epoch_train_loss)
            
            if (epoch + 1) % 100 == 0:
                print(f'Epoch [{epoch+1}/{config.epochs}], Loss: {loss.item():.4f}')

            model.eval()
            running_test_loss = 0.0
            with torch.no_grad():
                total_loss = 0
                for input_batch, target_batch in test_loader:
                    outputs = model(input_batch)
                    loss = criterion(outputs, target_batch)
                    total_loss += loss.item()
                    running_test_loss += loss.item() * input_batch.size(0)
                
                epoch_test_loss = running_test_loss / len(test_loader.dataset)
                test_losses.append(epoch_test_loss)
                
                avg_loss = total_loss / len(test_loader)
                if (epoch + 1) % 10 == 0:
                    print(f'Average Test Loss: {avg_loss:.4f}')

                if avg_loss < best_loss:
                    best_loss = avg_loss
                    best_weights = copy.deepcopy(model.state_dict())
                    # print(f'New best model at epoch {epoch+1} with loss {best_loss: .4f}')
            wandb.log({"train loss": epoch_train_loss, "test loss": epoch_test_loss})

        torch.save(best_weights, 'best_model.pth')
        model.load_state_dict(torch.load('best_model.pth')) 

# Set up a sweep
sweep_config = {
    "method": "random",
    "name": "sweep_test",
    "metric": {"goal": "minimize", "name": "loss"},
    "parameters": {
        # "batch_size": {"values": [32, 64, 128, 256]},
        # "epochs": {"values": [100, 200, 300]},
        "epochs": {"values": [200]},
        # "lr": {"values": [0.0001, 0.001, 0.01]},
        "lr": {"values": [0.0001]},
        # "hidden_size": {"values": [50, 100, 200]}
        "hidden_size": {"values": [100]},
        # "dropout": {"values": [0.2, 0.5]}
        "dropout": {"values": [0.5]}
    },
    "run_cap": 1
}

if __name__ == "__main__":
    sweep_id = wandb.sweep(sweep=sweep_config, project="MLP_calibration")
    wandb.agent(sweep_id, function=train)

# Evaluate the model and compare predictions with actual values
def evaluate_model(model, data_loader, criterion):
    model.eval()  # Set the model to evaluation mode
    total_loss = 0
    total_samples = 0
    predictions = []
    actuals = []    

    with torch.no_grad():  # Disable gradient calculation
        for input_batch, target_batch in data_loader:
            outputs = model(input_batch)
            loss = criterion(outputs, target_batch)
            total_loss += loss.item() * input_batch.size(0)
            total_samples += input_batch.size(0)
            predictions.append(outputs.numpy())
            actuals.append(target_batch.numpy())    

    avg_loss = total_loss / total_samples
    predictions = np.vstack(predictions)  # Combine batches into a single array
    actuals = np.vstack(actuals)  # Combine batches into a single array
    return avg_loss, predictions, actuals
    
# criterion = RMSELoss()

# Evaluate on the test set
# test_loss, test_predictions, test_actuals = evaluate_model(model, test_loader, criterion)
# print(f'Test RMSE Loss: {test_loss:.4f}')

# Plot the predictions vs actual values
# plt.figure(figsize=(10, 5))
# for i in range(test_predictions.shape[1]):  # Plot each output dimension
#     plt.plot(test_predictions[:, i], label=f'Predictions (dim {i})')
#     plt.plot(test_actuals[:, i], label=f'Actuals (dim {i})')
# plt.legend()
# plt.show()

# weights = {}
# for name, param in model.named_parameters():
#     if(param.requires_grad):
#         weights[name] = param.data.numpy()

# print("fc1 weights:", weights['fc1.weight'])
# print('fc1 bias:', weights['fc1.bias'])
# print("fc2 weights:", weights['fc2.weight'])
# print('fc2 bias:', weights['fc2.bias'])

