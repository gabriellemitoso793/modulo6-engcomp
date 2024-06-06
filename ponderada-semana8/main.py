import numpy as np

class MLP:
    def __init__(self, input_size, hidden_size, output_size, learning_rate=0.1):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        self.learning_rate = learning_rate
        
        self.weights_input_hidden = np.random.rand(input_size, hidden_size)
        self.bias_hidden = np.random.rand(hidden_size)
        
        self.weights_hidden_output = np.random.rand(hidden_size, output_size)
        self.bias_output = np.random.rand(output_size)

    def _sigmoid(self, x):
        return 1 / (1 + np.exp(-x))
    
    def _sigmoid_derivative(self, x):
        return x * (1 - x)

    def forward_pass(self, X):
        self.hidden_input = np.dot(X, self.weights_input_hidden) + self.bias_hidden
        self.hidden_output = self._sigmoid(self.hidden_input)
        
        self.final_input = np.dot(self.hidden_output, self.weights_hidden_output) + self.bias_output
        self.final_output = self._sigmoid(self.final_input)
        
        return self.final_output

    def backward_pass(self, X, y, output):
        output_error = y - output
        output_delta = output_error * self._sigmoid_derivative(output)
        
        hidden_error = output_delta.dot(self.weights_hidden_output.T)
        hidden_delta = hidden_error * self._sigmoid_derivative(self.hidden_output)
        
        # Atualizando os pesos e biases
        self.weights_hidden_output += self.hidden_output.T.dot(output_delta) * self.learning_rate
        self.bias_output += np.sum(output_delta, axis=0) * self.learning_rate
        
        self.weights_input_hidden += X.T.dot(hidden_delta) * self.learning_rate
        self.bias_hidden += np.sum(hidden_delta, axis=0) * self.learning_rate

    def train(self, X, y, epochs=10000):
        for _ in range(epochs):
            output = self.forward_pass(X)
            self.backward_pass(X, y, output)

    def predict(self, X):
        return self.forward_pass(X)

# Dados de entrada para a porta XOR
X = np.array([[0, 0],
              [0, 1],
              [1, 0],
              [1, 1]])

# Saída esperada
y = np.array([[0],
              [1],
              [1],
              [0]])

# Inicializando a rede neural
mlp = MLP(input_size=2, hidden_size=2, output_size=1, learning_rate=0.1)

# Treinando a rede
mlp.train(X, y, epochs=10000)

# Testando a rede
for x in X:
    print(f"Input: {x} Output: {mlp.predict(x)}")
