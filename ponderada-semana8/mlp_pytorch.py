import torch
import torch.nn as nn
import torch.optim as optim
import pytorch_lightning as pl

# Definindo o modelo MLP usando PyTorch Lightning
class MLP(pl.LightningModule):
    def __init__(self, input_size, hidden_size, output_size):
        super(MLP, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(hidden_size, output_size)
        self.sigmoid = nn.Sigmoid()
        self.criterion = nn.MSELoss()

    def forward(self, x):
        hidden = self.relu(self.fc1(x))
        output = self.sigmoid(self.fc2(hidden))
        return output

    def training_step(self, batch, batch_idx):
        x, y = batch
        outputs = self(x)
        loss = self.criterion(outputs, y)
        self.log('train_loss', loss)
        return loss

    def configure_optimizers(self):
        optimizer = optim.SGD(self.parameters(), lr=0.1)
        return optimizer

# Dados de treinamento XOR
train_data = torch.tensor([[0, 0], [0, 1], [1, 0], [1, 1]], dtype=torch.float32)
target = torch.tensor([[0], [1], [1], [0]], dtype=torch.float32)
dataset = torch.utils.data.TensorDataset(train_data, target)
train_loader = torch.utils.data.DataLoader(dataset, batch_size=1, shuffle=True)

# Parâmetros do modelo
input_size = 2
hidden_size = 2
output_size = 1

# Inicializando o modelo
model = MLP(input_size, hidden_size, output_size)

# Treinando o modelo
trainer = pl.Trainer(max_epochs=50000, log_every_n_steps=1000)
trainer.fit(model, train_loader)

# Testando o modelo
with torch.no_grad():
    for datapoint in train_data:
        output = model(datapoint)
        predicted_class = 1 if output >= 0.5 else 0
        print(f'Entrada: {datapoint.numpy()}, Saída Prevista: {output.item()}, Classe Prevista: {predicted_class}')

