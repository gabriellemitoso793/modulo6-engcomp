# Reconhecimento de Algarismos Manuscritos

## Descrição
Este projeto utiliza uma rede neural convolucional (CNN) para reconhecer algarismos manuscritos utilizando o conjunto de dados MNIST. O backend é implementado em Flask e possui uma interface web para upload de imagens.

## Instalação
Clone o repositório e instale as dependências:

```bash
git clone <URL_DO_REPOSITORIO>
cd <NOME_DO_REPOSITORIO>
pip install -r requirements.txt
```

## Como rodar

### Treinamento do Modelo

Execute o script de treinamento para treinar o modelo e salvar os pesos:

```bash
python3 models/train_model.py
```
### Executando o Backend

Inicie o servidor Flask:

```bash
python app.py
```