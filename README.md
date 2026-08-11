# Tello ArUco Landing — Pouso Autônomo em Marcador ArUco

[![Assista à demonstração](https://img.youtube.com/vi/8bLkRrrKNyQ/maxresdefault.jpg)](https://youtu.be/8bLkRrrKNyQ)

Projeto em ROS 2 que controla um drone DJI Tello para **identificar, se alinhar e pousar automaticamente sobre um marcador ArUco**, usando visão computacional (OpenCV) e uma câmera adaptada para olhar para baixo.

## Objetivo

O objetivo do projeto é fazer o drone executar, de forma autônoma, o ciclo completo de um pouso de precisão:

1. **Identificar** um marcador ArUco no campo de visão da câmera;
2. **Calibrar/alinhar** sua posição sobre o marcador, corrigindo o deslocamento lateral e frente/trás;
3. **Pousar** o mais próximo possível do centro do marcador.

Todo o comportamento do drone (decolagem, movimentos, ativação da detecção, etc.) é definido através de um **arquivo de missão em JSON**, sem precisar recompilar o código para cada teste — basta editar o JSON e rodar o node de novo.

## Como funciona

O node roda um loop de detecção de ArUco a ~30 FPS em paralelo à execução da missão (que roda em uma thread separada). Assim que um marcador com ID válido é identificado, enquanto o drone está voando:

1. A missão em andamento é **cancelada imediatamente** — nenhum novo comando da lista JSON é enviado a partir desse momento.
2. Começa a **rotina de pouso alinhado**: a cada ciclo, o código calcula o desvio (offset) do centro do marcador em relação ao centro da imagem e corrige um eixo por vez — primeiro o deslocamento lateral (esquerda/direita), depois frente/trás — em passos fixos de movimento.
3. Quando o drone está alinhado dentro de uma tolerância aceitável, ele desce um pouco e reavalia, repetindo o processo até estar alinhado e baixo o suficiente para pousar.
4. Existe um **tempo limite de segurança**: se o drone não conseguir se alinhar completamente dentro desse prazo, ele pousa onde estiver, em vez de tentar indefinidamente (ver seção de limitações abaixo sobre o motivo disso ser necessário).
5. Se o marcador sair do campo de visão da câmera durante o alinhamento, o código tenta desfazer o último movimento (na tentativa de reencontrá-lo); se mesmo assim continuar sem detectar por tempo demais, o drone pousa onde estiver, por segurança.

Todos os comandos do Tello são serializados por um lock (`_tello_lock`), já que o drone só processa um comando por vez — evitando conflitos entre a thread de detecção e a thread de execução da missão.

## Formato do arquivo de missão (JSON)

O JSON é uma lista de objetos, cada um representando um comando, executado em ordem:

```json
[
  {"comando": "decolar"},
  {"comando": "subir", "valor": 60},
  {"comando": "aruco"},
  {"comando": "sair"}
]
```

### Comandos disponíveis

| Comando | Valor | Descrição |
|---|---|---|
| `decolar` | — | Decola |
| `pousar` | — | Pousa imediatamente |
| `frente` | 20–500 (cm) | Move para frente |
| `tras` | 20–500 (cm) | Move para trás |
| `esquerda` | 20–500 (cm) | Move para a esquerda |
| `direita` | 20–500 (cm) | Move para a direita |
| `subir` | 20–500 (cm) | Sobe |
| `descer` | 20–500 (cm) | Desce |
| `girar` | 1–360 (graus) | Gira no eixo (negativo = anti-horário) |
| `aruco` | — | Ativa a detecção de ArUco. **Não é bloqueante** — a missão continua imediatamente para o próximo comando. Fica ativa até o fim do programa. Ao identificar um marcador válido, cancela a missão e inicia a rotina de pouso alinhado. |
| `camera` | — | Abre uma janela mostrando a imagem da câmera em tempo real (uso opcional, só para debug visual) |
| `esperar` | segundos | Pausa a execução da missão pelo tempo informado |

## Acompanhamento em tempo real

O node publica tudo o que está acontecendo (comandos executados, identificação do marcador, decisões de pouso, progresso do alinhamento) em um tópico ROS 2 dedicado:

```bash
ros2 topic echo /tello/status
```

A imagem da câmera também é publicada em `/tello/image_raw`, para uso por outros nodes ou ferramentas como `rqt_image_view`.

## Por que o dicionário ArUco `4X4_50`

O OpenCV oferece vários dicionários de marcadores ArUco, variando em dois eixos: o tamanho da grade interna (4x4, 5x5, 6x6...) e a quantidade de IDs disponíveis (50, 100, 250...). A escolha de `DICT_4X4_50` não foi arbitrária — considerou o hardware disponível:

- **Grade 4x4 (menos bits)**: quanto menor a grade interna do marcador, menos pixels a câmera precisa resolver claramente para decodificar o ID corretamente. Isso é importante porque a câmera do Tello tem resolução e qualidade inferiores às de uma webcam comum, e ainda passa pela adaptação física (espelho) que já introduz alguma perda de nitidez. Dicionários maiores (5x5, 6x6, 7x7) até oferecem mais robustez teórica contra erros de leitura, mas exigem uma imagem bem mais nítida/próxima para serem lidos — o que não é realista nesse setup. Na prática, os padrões de detecção do OpenCV já vieram mais rígidos que o necessário para essa câmera (como vimos durante os testes), então usar uma grade maior só pioraria a taxa de detecção.
- **Apenas 50 IDs**: o projeto não precisa de centenas de marcadores diferentes — a lista de pouso válida (`IDS_VALIDOS_POUSO`) usa só 5 IDs (1 a 5). Um dicionário de 50 já sobra bastante margem, sem pagar o custo de complexidade adicional de um dicionário maior (mais bits para decodificar = mais chance de erro na leitura).

Em resumo: **priorizamos confiabilidade de leitura em condições de câmera ruim/distância maior, em vez de ter uma quantidade enorme de IDs únicos**, que o projeto simplesmente não precisa.

### Sobre o tamanho físico do marcador impresso

O tamanho do marcador impresso (o valor usado em `MARKER_LENGTH_M` nos scripts de calibração/pose) também é um trade-off direto:

- **Marcador maior**: fica visível e decodificável de uma altura maior, dando mais margem para o drone iniciar o alinhamento de longe. Em compensação, ocupa mais espaço físico no chão/plataforma de pouso.
- **Marcador menor**: só é confiavelmente identificado quando o drone já está bem próximo/baixo, o que reduz a janela de tempo disponível para a rotina de alinhamento agir antes do pouso.

Como o Tello já sofre com resolução de câmera limitada, optamos por um marcador **grande o suficiente para ser identificado com folga de altura**, dando tempo real para a rotina de correção de posição atuar antes da fase final de descida — em vez de um marcador pequeno, que só apareceria tarde demais no processo.

## Limitações conhecidas

- **Passo mínimo de movimento de 20 cm**: o Tello não aceita comandos de deslocamento menores que 20 cm — é uma limitação do próprio firmware/SDK, não do código. Isso impede ajustes finos durante o alinhamento; o drone corrige em "saltos" de 20 cm, o que pode levar a overshoot (passar do ponto ideal) especialmente em alturas baixas.
- **Tempo limite de pouso**: justamente por causa do passo mínimo de 20 cm, existe risco do drone ficar preso em um loop de correção sem nunca convergir para dentro da tolerância desejada. Por isso foi implementado um tempo máximo (`TEMPO_LIMITE_POUSO_S`) — se estourado, o drone pousa onde estiver, por segurança, em vez de arriscar ficar voando indefinidamente.
- **Câmera com adaptação física**: o hardware utilizado tem uma adaptação (espelho a 45°) para permitir que a câmera frontal do Tello enxergue para baixo. Isso introduz inversões que precisam ser compensadas via software (flip de imagem e inversão dos comandos de frente/trás), configuráveis no topo do código.
- **Qualidade da câmera**: a câmera do Tello tem resolução e qualidade inferiores às de uma webcam comum, o que exige parâmetros de detecção mais tolerantes que o padrão do OpenCV para reconhecer os marcadores de forma confiável.
- **Sem calibração de câmera real**: a estimativa de posição hoje é baseada apenas no deslocamento em pixels do centro do marcador em relação ao centro da imagem — não há calibração real da câmera (matriz intrínseca/distorção), então não há estimativa precisa de distância real (metros) até o marcador.
- **Estabilidade de rede**: comandos e vídeo trafegam pela rede wifi própria do Tello; sinal fraco ou interferência pode causar atrasos ou, em casos extremos, acionar o pouso automático de segurança do próprio drone.

## Melhorias futuras

- Calibração real da câmera (matriz intrínseca + coeficientes de distorção) para estimativa de pose via `solvePnP`, permitindo saber a distância real até o marcador em metros.
- Alinhamento mais suave, usando controle de velocidade contínuo (RC control) em vez de comandos de deslocamento discretos de 20 cm.
- Ajuste dinâmico da tolerância de alinhamento conforme a altura do drone (quanto mais baixo, menor a tolerância necessária em pixels para equivaler à mesma precisão em cm reais).
- Melhorar a robustez da detecção em condições de pouca luz.
- Testes com múltiplos marcadores para pouso em diferentes plataformas na mesma missão.

## Requisitos

- ROS 2 (testado no Humble)
- Python 3.10+
- `djitellopy`
- `opencv-contrib-python`
- `cv_bridge`
