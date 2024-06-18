# Reconhecimento de Algarismos Manuscritos

## Descrição
Este projeto utiliza uma rede neural convolucional (CNN) para reconhecer algarismos manuscritos utilizando o conjunto de dados MNIST. O backend é implementado em Flask e possui uma interface web para upload de imagens.

## Instalação
Clone o repositório e instale as dependências:

```bash
git clone https://github.com/gabriellemitoso793/modulo6-engcomp.git
cd ponderada-semana9
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

### Vídeo demonstrativo

Link treinamento do modelo: https://drive.google.com/file/d/1BGwU3-Mj6pHqYM8fCUFTL_FouY7SDael/view?usp=sharing

Link para demonstração na interface: https://drive.google.com/file/d/1kk-m4HhH__SUaqf_3p331CQy0vo1ALv9/view?usp=sharing