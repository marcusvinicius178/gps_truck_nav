# Terceira revisão — demonstração observável e validade experimental

**03/11/2026. PR #2 DRAFT, sem merge. Gate 0 e Gate 1: NOT_YET_VERIFIED.** Esta entrega incorpora a diretriz de prioridade: avançar à observação funcional e preparar atuação tecnicamente consistente, preservando a baseline histórica. Não transformar ausência de warning ou uma contagem de PASS em melhoria de navegação.

## Evidência recebida e fatos preservados

Foram lidos integralmente o relatório `nav2_revisao_independente_a21403f.md` e os 120 arquivos do ZIP `nav2_phase_a_second_review_evidence.zip` antes das alterações relevantes. ZIP SHA-256 `e2ffc8d0c875abc816cbccb5470ff25902e430691aba75e51cafb4c14445221d`, 1969956 bytes descompactados; commit `a21403fcc67e027e29010d29f6339a580dfb7076`, imagem `sha256:b3fb15f5ad6b679f17a4e57ed795a00a59d4d5cf1213b1c28bd1363ae9df9e22`.

O host comprovou IMAGE_VALIDATED, OBSERVATIONS_COLLECTED, captura PARTIAL e diagnóstico INCOMPLETE: **69 PASS, zero FAIL, 3 UNKNOWN e 10 BLOCKED_REVIEW**. Isso não é uma falha de navegação demonstrada; nenhuma navegação foi executada. Cleanup do container próprio foi confirmado.

Permanecem aprovados dentro da janela observada: seis pais TF articulados com RSP único; odom e juntas preservados; frescor com clock da mesma janela; headers GPS/IMU; consistência dos lever arms; costmaps; duas BTs Ackermann; sete lifecycle nodes Nav2 e duas costmaps ACTIVE; parâmetros obrigatórios Smac/MPPI e identidade das suas bibliotecas mapeadas. Não há mudança de TF, sensores, costmaps, BTs, launches, datum ou geometria nesta revisão.

Os resíduos GNSS/TF de aproximadamente 1,397 mm, 1,377 mm e 0,021 mm são consistência interna com sensores compartilhados. Não provam erro absoluto contra a geometria do mundo, nem o tempo interno de compensação do navsat. As covariâncias permanecem intactas e pendentes de política fundamentada.

## P0 — instrumentação confiável

### Descoberta Gazebo

O log registra `gzserver` PID 87, mas o observador anterior filtrava o basename de `/proc/<pid>/exe` por igualdade literal. O source Gazebo Classic 11.10.2 instala executáveis com propriedade CMake VERSION; isso sustenta a hipótese de basename versionado. O ZIP não contém o readlink desse PID, portanto o nome exato ainda precisa ser observado.

O coletor agora aceita estritamente `gzserver` ou `gzserver-11.N.N`, separa nome real de papel do processo, guarda inventário PID→exe e correlaciona PIDs do log congelado. Revalida starttime/inode durante a captura. Arquivo deletado, permissão, PID reutilizado, inode/hash divergente ou ausência do PID esperado não viram PASS. `PID_MATCH_ONLY` explicita que o log antigo não contém starttime para provar a mesma encarnação desde o lançamento. A próxima coleta precisa mostrar a identidade do executável e das duas bibliotecas Gazebo realmente mapeadas.

Fontes primárias: [Gazebo CMakeLists 11.10.2](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/gazebo/CMakeLists.txt), [macro de instalação/versionamento](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/cmake/GazeboUtils.cmake), [CMake VERSION](https://cmake.org/cmake/help/latest/prop_tgt/VERSION.html).

### Opcionais ausentes versus RPC com falha

O relatório recebido tem 107 valores obrigatórios/observados válidos e 21 nomes opcionais ausentes, que antes geravam 42 jobs ERROR e três tentativas por job. Não se infere UNDECLARED somente porque um RPC voltou vazio. A nova classificação exige ListParameters recursivo, consistente com os obrigatórios, e ambas as respostas Get/Describe singleton corroborantes dentro do prazo. Contradição, lista incompleta, erro de transporte ou resposta tardia continuam inconclusivos.

Schema 2 separa `collection_status` (resultados resolvidos), `readback_collection_status` (respostas completas) e `required_collection_status` (obrigatórios). Uma ausência comprovada pode completar a investigação sem fabricar um valor. `UNSUPPORTED_VERSION` exige a atribuição de source/versão/identidade já documentada. Nenhum nome ausente ou sem consumidor se torna elegível ao tuning. `ACTUALLY_USED` permanece NOT_VERIFIED. O replay offline dos dados recebidos preservou os 107 valores, resolveu os 21 nomes com uma tentativa e reduziu de 146 para 62 RPCs; esse replay não é execução no host.

## P1 — atuação preparada, ainda desconectada

`actuation_contract.py` é um núcleo Python puro, sem ROS, I/O ou publisher. Calcula a inversa para a semântica do plugin, verifica ambos os joints, rejeita curvatura irrealizável, trata ré/zero/histerese e limita velocidade/yaw conjuntamente preservando curvatura quando essa política é explicitamente escolhida. Um supervisor determinístico modela autoridade, identidade, relógio monotônico, expiração e falhas travadas. Não é um watchdog operacional; todos os resultados mantêm `runtime_authorized=false` e não comprovam parada física.

Não foram inventados defaults para faixa de parada, histerese, timeout ou limites físicos. A geometria auditada continua estática: L≈6,8165 m, limite ideal de raio≈13,09465 m, sem reconciliação ao raio 11,6 m ou ao ponto cinemático base/eixo traseiro. Não há adaptador conectado, remapeamento novo da planta, patch no Gazebo/Nav2 instalado ou liberação automática de ré.

`actuation_validation_protocol.md` define a sequência futura: revisar identidade e referências → validar autoridade e interrupção final → ensaios controlados de baixa velocidade → Gate 0 supervisionado → 20 km/h em condições aprovadas → 25 km/h somente após resultados suficientes. Cancelar a action não equivale a parar. A interrupção deve ser comprovada por estado independente da planta e também cobrir morte do adaptador/smoother. O protocolo prepara seleção de objetivo, visualização do plano, cancelamento e critérios de aborto, mas nenhuma dessas ações de movimento é executada nesta entrega.

## P2 — perfis prospectivos de velocidade

A investigação Git em `speed_limits_provenance.md` encontrou MPPI 10, depois 20, RPP 22 ativo em uma revisão e MPPI 25 reativado em janeiro de 2026. O smoother ±8,33 m/s permaneceu nessas revisões. Não há evidência nas revisões percorridas de que 25 fosse typo de 2,5. Títulos de commits como “works” não substituem logs de navegação; o comportamento histórico real não é deduzido apenas dos YAMLs públicos.

Os YAMLs completos `navigation_profiles/20kmh.yaml` e `25kmh.yaml` alteram somente cinco folhas: `FollowPath.vx_max` e componentes x/yaw dos limites superior/inferior do smoother. Usam 50/9 e 125/18 m/s como tetos prospectivos retos, respectivamente, e os limites MPPI existentes de ré −2,25 m/s e yaw ±0,4 rad/s. Não alteram raios, modelos DUBIN/Ackermann, pesos, acelerações, covariâncias, planta, BT ou costmaps. Manifest e gerador verificam hash canônico, diferenças permitidas e bytes completos.

São propostas de envelope, não velocidades constantes em curva ou capacidades físicas aprovadas. Com yaw cap 0,4, os quocientes v/ω nos dois tetos são 13,889 e 17,361 m; curvas mais fechadas exigem menor velocidade. Diminuir vx_max não torna o raio 11,6 realizável, e `scale_velocities=true` sozinho não corrige clamps independentes ou bypass do behavior_server. Essa opção histórica não foi alterada.

O helper aceita `probe --navigation-profile 20kmh|25kmh` exclusivamente para leitura sem goals. O launch recebe o YAML completo e o observador recebe a mesma seleção; os parâmetros tipados esperados são derivados desse arquivo verificado. O default permanece `canonical`, isolando a correção de instrumentação. Agora há GetParameters explícito também para os limites/acelerações/timeout do smoother. `speed_chain.json` registra envelopes, horizonte, raio cinemático no teto e publishers que contornam o smoother; não confunde o mínimo nominal dos caps com velocidade física medida.

## P3 — visualização ao vivo, somente observação

Gazebo Classic e RViz executam dentro de um container próprio, com Xvfb. Um visor HTTP local exibe a tela por imagens atualizadas; não transmite teclado, mouse, serviços ou goals para o container. Isso evita chamar um gzclient nativo interativo de “read-only”: ele normalmente permite mover modelos e resetar física. Neste primeiro modo não há pan/zoom interativo. O RViz também omite ferramentas/painéis de ação.

`visual_observer.py` fornece start/status/serve/stop, exige imagem e fonte correspondentes a build/probe e usa duração limitada. Rede do container continua `none`, sem portas publicadas, socket X11, GPU, ambiente Humble ou instalações no host. O servidor HTTP Python do host escuta somente localhost e não possui rota de escrita; a URL é impressa pelo comando serve. Saída do servidor, timeout ou comando stop encerram somente o container identificado por ID/labels, preservando logs. Captura de PNG é opt-in e limitada. O modo headless permanece disponível.

O mundo visual deriva do mesmo mundo prospectivo e muda somente o bloco de câmera/GUI, com prova da diferença. A camada de imagem existente copia os novos scripts/configurações; não há APT novo. A nova prova deve confirmar janelas Gazebo/RViz, Arocs, sensores, TF, footprint, costmaps e pose. Planos e trajetórias só existirão quando houver uma ação de planejamento posteriormente autorizada; não criar uma trajetória fictícia para preencher a tela.

## Comparação justa e critérios do próximo retorno

Há três identidades diferentes: fontes históricas preservadas; baseline diagnóstica a21403f observada sem movimento; variante prospectiva de atuação/perfis. Não chamar reparo de planta/interface de ganho de fine-tuning. Registrar hashes, versões, mundo, interface, regras de parada, sementes/reset, objetivos e métricas antes de A/B. Após a planta ser aprovada, baseline de controle e candidato de tuning precisam usar a mesma planta/interface. Os 212 experimentos históricos, Overleaf e ambientes do host não são modificados.

| Escopo | Critério verificável |
|---|---|
| Instrumentação | Identidade Gazebo pelo processo correlacionado; opcionais ausentes distintos de falha RPC; obrigatórios continuam completos. |
| Regressões já resolvidas | TF/frames/juntas/odom, costmaps, duas BTs e lifecycle mantêm os resultados aprovados. |
| Perfis | Hash/manifest válidos; Get tipado mostra 5,555555… ou 6,944444… nos dois pontos do caminho principal; yaw/ré coerentes; bypass ainda explicitamente bloqueado. |
| GUI | Janelas e dados visíveis, somente leitura, timeout/stop próprios comprovados; nenhuma mensagem de atuação originada pelo visor. |
| Preparação de atuação | Testes offline de equações/falhas aprovados; geometria, referência, roteamento e interrupção final ainda exigem prova. |
| Gate 0 | Autorização posterior, atuação validada, percepção/localização suficientes, objetivo limitado, plano observado, parada/cancelamento medidos e métricas. Não atingido aqui. |
| Gate 1/tuning | Reset e dois trials automáticos comprovados somente após Gate 0; knobs consumidos e planta fixa. Não iniciado. |

**354 testes PASS**, auditoria estrutural PASS e sintaxe de 24 blocos Bash/sete Python embutidos nos três documentos operacionais PASS. Logs e resumo estão em `evidence/third_review_*`. São testes offline, incluindo núcleo C++ sem ROS, fixtures e serviços HTTP locais artificiais; não substituem build/GUI/probe no ipt-sim-01. A próxima execução está na seção **2.4** do runbook. Nenhum goal, Twist, missão, Optuna, fine-tuning ou Virtual Lanes foi executado.
