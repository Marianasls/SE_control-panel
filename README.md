# Panel de Controle Interativo para Restaurante Universitário (RU)

Este projeto implementa um painel de controle interativo para um Restaurante Universitário (RU), utilizando Raspberry Pi Pico, display OLED, LEDs RGB, buzzer e botões físicos. O sistema é desenvolvido em C, utilizando FreeRTOS para multitarefa, mutex e semáforos para controle de concorrência.

## Funcionalidades

- **Contagem de Usuários:** Controle do número de pessoas presentes no RU, com limite máximo configurável.
- **Display OLED:** Mostra em tempo real o número de pessoas e a cota restante.
- **LED RGB:** Indica visualmente o status de ocupação:
  - Azul: vazio
  - Verde: disponível
  - Amarelo: quase cheio
  - Vermelho: lotado
- **Buzzer:** Sinal sonoro para eventos como lotação máxima ou reset.
- **Botões físicos:** 
  - Entrada (Botão A incrementa usuários)
  - Saída (Botão B decrementa usuários)
  - Reset (Botão Joystick zera a contagem)
- **FreeRTOS:** Uso de tarefas, mutex e semáforos para garantir concorrência segura entre os componentes.

## Componentes Utilizados

- **Microcontrolador:** Raspberry Pi Pico (RP2040)
- **Display:** OLED SSD1306 (I2C)
- **LEDs:** RGB
- **Buzzer:** PWM
- **Botões:** 3 (Entrada, Saída, Reset/Joystick)
- **Sistema Operacional:** FreeRTOS

## Como Executar o Projeto

### Pré-requisitos

- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [FreeRTOS Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel)
- CMake (>=3.13)

### Passos para Compilar e Executar

1. **Clone o repositório e dependências:**
   ```sh
   git clone <este-repo>
   cd Panel_Control
   # Certifique-se de que o pico-sdk e o FreeRTOS-Kernel estejam nos caminhos corretos
   ```

2. **Configure o ambiente do Pico SDK:**
   ```sh
   export PICO_SDK_PATH=/caminho/para/pico-sdk
   ```

3. **Crie a pasta de build e gere os arquivos:**
   ```sh
   mkdir build
   cd build
   cmake ..
   ```

4. **Compile o projeto:**
   ```sh
   cmake --build .
   ```

5. **Grave o firmware no Raspberry Pi Pico:**
   - Conecte o Pico em modo BOOTSEL e copie o arquivo `.uf2` gerado na pasta `build` para o dispositivo USB do Pico.


## Observações

- O projeto utiliza mutex para proteger o acesso ao display OLED e semáforos para controlar a contagem de usuários e reset.
- O código é facilmente adaptável para outros ambientes de controle de acesso.


**Desenvolvido para fins acadêmicos e de demonstração de sistemas embarcados com FreeRTOS.**