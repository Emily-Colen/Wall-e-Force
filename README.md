<div align="center">

# 🤖 WALL-E FORCE

#Link do repositório Gitub: https://github.com/Emily-Colen/Wall-e-Force.git

### Robô Sumô Autônomo | ESP32-WROOM-32

**Um robô compacto de 10 × 10 cm projetado para localizar, perseguir e empurrar adversários em um dojô de 60 cm de diâmetro.**

![Linguagem C](https://img.shields.io/badge/Linguagem-C-00599C?style=for-the-badge&logo=c&logoColor=white)
![ESP32](https://img.shields.io/badge/Placa-ESP32--WROOM--32-E7352C?style=for-the-badge&logo=espressif&logoColor=white)
![ESP-IDF](https://img.shields.io/badge/Framework-ESP--IDF-333333?style=for-the-badge)
![PlatformIO](https://img.shields.io/badge/IDE-PlatformIO-F5822A?style=for-the-badge&logo=platformio&logoColor=white)

</div>

---

## 📌 Sobre o projeto

O **WALL-E FORCE** é um projeto de robótica embarcada que utiliza uma **ESP32-WROOM-32** para controlar os motores, localizar o adversário por ultrassom e identificar os limites da arena com sensores infravermelhos.

O software foi desenvolvido em **C com ESP-IDF** e utiliza uma **máquina de estados** para organizar os comportamentos de busca, ataque e evasão. Os movimentos são temporizados sem pausas longas, e os infravermelhos utilizam interrupções com verificação periódica das leituras.

### 📐 Especificações

| Característica | Descrição |
|---|---|
| Categoria | Robô sumô autônomo |
| Dimensões do robô | 10 × 10 cm |
| Arena | Dojô circular de 60 cm de diâmetro |
| Controlador | ESP32-WROOM-32 (placa de 38 pinos) |
| Linguagem | C |
| Framework | ESP-IDF 5.x |
| Ambiente de desenvolvimento | PlatformIO / VS Code |

## 🔧 Hardware utilizado

- **1 × ESP32-WROOM-32** (38 pinos)
- **1 × HC-SR04 frontal** para detectar o adversário
- **3 × TCRT5000** para detectar as bordas: esquerdo, direito e traseiro
- **1 × ponte H TB6612FNG** para controlar dois motores
- **2 × motores DC** e rodas
- Chassi, alimentação e conexões elétricas

### 🔌 Pinagem

| Dispositivo | Sinal | GPIO ESP32 |
|---|---|---:|
| HC-SR04 frontal | TRIG | 32 |
| HC-SR04 frontal | ECHO | 34 |
| TCRT5000 esquerdo | Saída digital | 36 |
| TCRT5000 direito | Saída digital | 39 |
| TCRT5000 traseiro | Saída digital | 18 |
| Ponte H — motor esquerdo | AIN1 / AIN2 / PWMA | 25 / 26 / 27 |
| Ponte H — motor direito | BIN1 / BIN2 / PWMB | 23 / 22 / 21 |

> ⚠️ **Segurança elétrica:** o HC-SR04 convencional é alimentado com 5 V, mas sua saída **ECHO precisa ser adaptada para 3,3 V** antes de chegar ao ESP32. Todos os dispositivos devem compartilhar uma referência GND comum. Os GPIOs 34, 36 e 39 são somente de entrada; 36 e 39 não possuem pull-up/pull-down internos.

## 🧠 Estratégia de combate

O programa utiliza quatro estados principais:

| Estado | Ação |
|---|---|
| `BUSCANDO` | Gira aproximadamente sobre o próprio eixo para localizar o adversário |
| `ATACANDO` | Avança com os dois motores para empurrar o adversário |
| `EVADINDO` | Corrige a trajetória quando identifica a borda |
| `PARADO_SEGURANCA` | Para diante de uma condição de borda que impede uma evasão segura |

### Prioridades

1. **Borda:** os sensores infravermelhos têm prioridade, inclusive durante o ataque.
2. **Adversário detectado:** o ultrassônico frontal inicia o ataque se a distância for válida e estiver dentro do alcance configurado.
3. **Sem adversário:** o robô continua a busca girando.

### ⚙️ Parâmetros da versão atual

| Parâmetro | Valor |
|---|---:|
| Distância para iniciar ataque | 35 cm |
| Distância para liberar alvo | 40 cm |
| Velocidade de busca (PWM) | 150 / 255 |
| Velocidade de ataque (PWM) | 255 / 255 |
| Velocidade de giro de evasão (PWM) | 180 / 255 |
| Velocidade de recuo (PWM) | 220 / 255 |
| Intervalo entre disparos ultrassônicos | 65 ms |
| Tolerância a ECHO inválido | 100 ms |
| Recuo de evasão (valor inicial) | 110 ms |
| Giro de evasão (valor inicial) | 140 ms |

> Esses valores são pontos de partida e precisam ser calibrados no robô físico. A busca está configurada com **dois motores em sentidos opostos** (`GIRO_BUSCA_UM_MOTOR = 0`).

## 📁 Estrutura do projeto

Um projeto **PlatformIO com ESP-IDF** pode ser organizado assim:

```text
Wall-e-Force/
├── src/
│   └── main.c
├── platformio.ini
├── CMakeLists.txt
├── .gitignore
└── README.md
```

> Se estiver usando o projeto ESP-IDF sem PlatformIO, o arquivo principal normalmente fica em `main/main.c`, acompanhado dos arquivos `CMakeLists.txt` correspondentes.

## 🚀 Como executar

1. Instale o **Visual Studio Code** e a extensão **PlatformIO IDE**.
2. Abra a pasta do projeto no VS Code.
3. Confira o arquivo `platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = espidf
monitor_speed = 115200
```

4. Conecte a ESP32 ao computador por USB.
5. Compile e grave o firmware usando **Build** e **Upload** do PlatformIO.
6. Abra o **Serial Monitor** em **115200 baud** para acompanhar as leituras e os estados do robô.

## 🧪 Testes e calibração

Antes de colocar o robô no dojô:

- [ ] Confirmar o sentido de rotação de cada motor com as rodas suspensas
- [ ] Verificar se o HC-SR04 mede corretamente com os motores ligados
- [ ] Confirmar o nível lógico dos TCRT5000 sobre o piso e a faixa branca
- [ ] Ajustar a altura e o posicionamento dos sensores de borda
- [ ] Testar recuo e giro de evasão sem risco de queda
- [ ] Calibrar as distâncias de ataque e os tempos de movimentação
- [ ] Conferir o limite de peso e as dimensões exigidos pela competição

## ⚠️ Cuidados importantes

- **Não conecte o ECHO de 5 V diretamente ao ESP32.** Use um divisor resistivo ou conversor de nível lógico.
- **Não alimente os motores pelo pino 3,3 V da ESP32.** Utilize alimentação apropriada para a ponte H e para os motores.
- Mantenha fios de sinal dos sensores afastados dos cabos dos motores sempre que possível.
- Valide a polaridade dos motores caso tenha invertido os fios `AO1/AO2` ou `BO1/BO2`.
- Evite testes de movimentação sem supervisão: o robô pode sair da arena durante a calibração.

## 📚 Referência didática

O projeto foi desenvolvido com base em práticas de programação embarcada em ESP32, incluindo configuração de GPIOs, FreeRTOS, controle PWM e medição do HC-SR04 pelo tempo do sinal ECHO. A rotina ultrassônica utiliza pulso TRIG de aproximadamente 10 µs e timeout para impedir espera infinita.

---

<div align="center">

**WALL-E FORCE** ⚡  
*Detectar. Buscar. Atacar. Permanecer na arena.*

</div>
