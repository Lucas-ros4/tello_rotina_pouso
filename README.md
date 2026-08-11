# Tello ArUco Landing — Pouso Autônomo em Marcador ArUco

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

## Vídeo de demonstração

*(link do vídeo aqui)*

## Requisitos

- ROS 2 (testado no Humble)
- Python 3.10+
- `djitellopy`
- `opencv-contrib-python`
- `cv_bridge`
