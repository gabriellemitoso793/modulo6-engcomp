import numpy as np

class MLP:
    def __init__(self, input_size, hidden_size, output_size, lr=0.1, num_epochs=30000):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        self.lr = lr
        self.num_epochs = num_epochs

        # Inicialização dos pesos
        self.weights_01 = np.random.uniform(size=(input_size, hidden_size))
        self.weights_12 = np.random.uniform(size=(hidden_size, output_size))
        self.b01 = np.random.uniform(size=(1, hidden_size))
        self.b12 = np.random.uniform(size=(1, output_size))

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def delsigmoid(self, x):
        return x * (1 - x)

    def forward(self, x):
        self.hidden = self.sigmoid(np.dot(x, self.weights_01) + self.b01)
        self.output = self.sigmoid(np.dot(self.hidden, self.weights_12) + self.b12)
        return self.output

    def backward(self, x, y):
        output_error = y - self.output
        output_delta = output_error * self.delsigmoid(self.output)

        hidden_error = output_delta.dot(self.weights_12.T)
        hidden_delta = hidden_error * self.delsigmoid(self.hidden)

        self.weights_12 += self.hidden.T.dot(output_delta) * self.lr
        self.weights_01 += x.T.dot(hidden_delta) * self.lr
        self.b12 += np.sum(output_delta, axis=0, keepdims=True) * self.lr
        self.b01 += np.sum(hidden_delta, axis=0, keepdims=True) * self.lr

    def train(self, train_data, target):
        for epoch in range(self.num_epochs):
            self.forward(train_data)
            self.backward(train_data, target)
            if (epoch+1) % 1000 == 0:
                loss = np.mean(np.square(target - self.output))
                print(f'Epoch [{epoch+1}/{self.num_epochs}], Loss: {loss:.4f}')

    def predict(self, x):
        return self.forward(x)

def main():
    train_data = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
    target = np.array([[0], [1], [1], [0]])

    mlp = MLP(input_size=2, hidden_size=2, output_size=1, lr=0.1, num_epochs=30000)
    mlp.train(train_data, target)

    for datapoint in train_data:
        output = mlp.predict(datapoint)
        predicted_class = 1 if output >= 0.5 else 0
        print(f"Entrada: {datapoint}, Saída Prevista: {output}, Classe Prevista: {predicted_class}")

if __name__ == "__main__":
    main()


# Código feito a partir do artigo "How Neural Networks Solve the XOR Problem"
# Link: https://towardsdatascience.com/how-neural-networks-solve-the-xor-problem-59763136bdd7
