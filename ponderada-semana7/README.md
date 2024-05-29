# Turtlebot Teleoperado

## Objetivo

O objetivo deste projeto é incrementar o sistema do turtlebot teleoperado para incluir conceitos de streaming de imagens.

## Descrição

Este é um programa que controla um robô usando o ROS (Robot Operating System) e exibe uma transmissão de webcam em uma interface web usando Streamlit.

## Instalação e Configuração

1. **Clone o Repositório:**
Primeiro é necesário clonar o repositório, para isso, acesse o terminal do seu computador e execute os seguintes comandos:

   ```bash
   git clone https://github.com/gabriellemitoso793/modulo6-engcomp.git
   cd modulo6-engcomp
   ```
2. **Construa o Projeto:**
Acesse a pasta 'meu_wokspace' no seu terminal e execute os seguintes comandos para construir o projeto e acionar o streamlit:

   ```bash
    cd modulo6-engcomp/ponderada-semana7/meu_workspace
    colcon build
    source install/local_setup.bash
    cd src/turtle_gabi/turtle_gabi
    streamlit run teste.py
   ```

3. **Abra um novo terminal**
Neste terminal execute os seguintes comandos para conectar ao robô:
    ```bash
    ssh grupo4@10.128.0.9
    cd main_ws
    source install/local_setup.bash
    cd launch
    ros2 launch launch.py
    ```

4. **Controles do Robô**

Você pode controlar o robô usando os botões W, A, S e D para aumentar ou diminuir a velocidade. O botão "Desligar" para o robô.

> W -> move o robô pare frente

> A -> move o robô para o lado esquerdo

> S -> move o robô para trás

> D -> move o robô para o lado direito

### Visualização da Webcam

A transmissão da webcam será exibida na interface do Streamlit, juntamente com a latência da comunicação.

### Importante

- Certifique-se de ter um ambiente ROS 2 configurado corretamente.
- Este programa foi projetado para fins educacionais e pode exigir ajustes para uso em diferentes ambientes ou robôs.


## Vídeo demonstrativo

Veja em: [Robô teleoperado com camera](https://drive.google.com/file/d/1CENBOx_pwky-RGVbB5C539e63TeFmo7W/view?usp=sharing)
