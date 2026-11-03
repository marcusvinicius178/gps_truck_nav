# Segunda revisão independente — Fase A, 03/11/2026

**PR #2 draft; sem merge. Gate 0 e Gate 1: NOT_YET_VERIFIED. Movimento não autorizado.** Foram lidos integralmente o relatório independente e os 115 arquivos do ZIP antes das alterações. Esta entrega implementa uma variante mínima de modelo e melhora a observação; não declara resolvidos os bloqueios que dependem da próxima execução ou de decisões físicas/metodológicas.

## Evidências recebidas e resultados preservados

| Artefato | Identidade |
|---|---|
| ZIP `nav2_phase_a_978ab23_review_evidence.zip` | SHA-256 `984c670c9da54488992c5ff21adb48d374542e8dfcefaca61e4ac591ddee1da7`; 115 arquivos, 762078 bytes descompactados |
| `nav2_revisao_independente_978ab23.md` | SHA-256 `4b51e5475da042f818091d1d80c57501c1447dee1334f6a129faff35d71d94f4` |
| Commit executado | `978ab2328c6cb378d6eea016027192a6cf1dbadf` |
| Imagem executada | `sha256:39d492f24ddd66639f58c527e6dd81cde3682983fc4e856de41a1ccfb0b5d09c` |
| Probe UTC | `2026-11-03T13:58:21.941036+00:00` a `2026-11-03T14:00:36.662604+00:00` |

O smoke passou em 60/60 verificações. O probe terminou `OBSERVATIONS_COLLECTED`, captura `COMPLETE`, diagnóstico `BLOCKED`, `mission_ready=false`: 49 PASS, 2 FAIL e 8 BLOCKED_REVIEW. Cleanup confirmou container próprio parado, sem OOM; isso não comprova reset experimental.

**Costmaps e BTs estão confirmadas nesse runtime.** Global 1000×300 m, local 200×120 m, rolling windows, footprint, frames e camadas corretos; sete lifecycle nodes Nav2 e duas costmaps ACTIVE. Não há evidência para reabrir esses reparos. Os YAMLs canônicos, todos os launches e as duas BTs permanecem byte a byte iguais ao commit recebido. Seus checks continuam como proteção contra regressão.

## 1. TF: causa e correção prospectiva

O ramo `publish_wheel_tf` do Gazebo Ackermann 3.7.0 publica seis filhos diretamente sob `base_footprint`; robot_state_publisher publica os mesmos filhos sob os pais articulados da URDF. São conflitos reais de árvore, distintos do problema de frescor. A variante completa `config/auto_tuning/models/arocs_truck_tf_sensors/model.sdf` desabilita somente esse ramo. `publish_odom`, `publish_odom_tf` e o plugin separado de `/joint_states` permanecem intactos. Não remover as rodas do diagnóstico: é obrigatório observar as seis arestas URDF com autoridade RSP, juntas finitas e avançando, `/odom` e `odom→base_footprint` preservados.

O manifest `tf_sensor_model_variant.json` fixa hashes, pais e alterações permitidas. O novo mundo muda apenas a URI do caminhão; datum, spawn, demais modelos e meshes não mudam. O helper seleciona esse mundo explicitamente e a imagem inclui seu diretório no `GAZEBO_MODEL_PATH`. O smoke verifica os arquivos realmente instalados. Chamadas históricas do launch continuam usando seus defaults.

### Frescor: o resultado antigo é inconclusivo

Os dois FAIL anteriores comparam carimbos do observador C++ com um `/clock` Python posterior. O fim C++ e a fotografia Python diferem em aproximadamente 0,624165 s. No C++, os últimos stamps de map→odom/odom→base são 130,383627/130,520280 s; o Python usa clock 131,610171 s, enquanto seu próprio Buffer tem transformações em 131,506848/131,583507 s. Isso não comprova atraso real nem falso positivo. Os contadores desiguais e o sleep fixo de 5 ms no loop anterior tornam backlog uma hipótese plausível, não uma causa demonstrada.

O observador schema 2 coleta `/clock` e TF na mesma janela; registra sequência, progressão/regressão, recepção monotônica, intervalos, cauda de 32 amostras e metadados RMW por aresta/GID. Remove o sleep fixo e usa espera do executor quando ocioso. Registra QoS e eventos de perda quando suportados; ausência desses eventos não prova perda zero. `assess_tf_timing` usa exclusivamente o clock desse observador. Os limites anteriores (1 s simulado, 0,25 s futuro e 2 s de silêncio de clock) não foram ampliados. Evidência antiga sem clock comum fica UNKNOWN, sem promoção retrospectiva a PASS.

## 2. Sensores, lever arms e covariâncias

Os sensores foram agregados ao link SDF base_footprint preservando seus offsets físicos; sem `frame_name`, o plugin usa esse link como header. A variante acrescenta explicitamente `gps_link` e `imu_link`, suas outras duas e únicas mudanças. Não move sensores nem altera ruído, datum, geometria ou fusão.

O coletor guarda amostras e timestamps inteiros de GNSS e `/odometry/gps`, consulta `/fromLL` e solicita TF no instante exato do fix. Compara antena com antena, base com base e compensação por rotação 3D do lever arm. Não usa latest/vizinho como substituto de amostra ausente. O serviço `/fromLL` não recebe tempo; o relatório distingue o stamp da entrada preservada dos tempos de requisição/resposta. PASS deste contrato significa comparabilidade temporal, **não** precisão geométrica, alinhamento da rota ou aprovação da compensação interna do navsat.

`localization_covariance_policy.md` documenta os offsets, zeros GPS, orientação IMU desconhecida e covariâncias derivadas do ruído SDF. O floor numérico 1e-9 do EKF não é calibração. Os zeros permanecem e continuam BLOCKED_REVIEW; nenhuma variância foi criada para obter PASS.

A inspeção do source robot_localization 3.7.1 encontrou ainda construção de `rclcpp::Time` a partir de segundos em um construtor de nanosegundos, com possível fallback para TF latest. A equivalência desse trecho ao binário instalado não está demonstrada. A nova coleta identifica executáveis/bibliotecas mapeados; não edita `/opt/ros/iron`. Uma eventual correção interna exigirá overlay e regressões próprios. Corrigir o header não encerra esse risco temporal.

## 3. Atuação: contrato estático, sem conexão ao veículo

MPPI entrega yaw rate; o upstream Gazebo Ackermann recebe esterçamento e inverte seu sinal conforme a velocidade. A biblioteca instalada foi identificada no ZIP, mas não seu inode carregado. A coleta nova correlaciona `/proc/<pid>/maps`, device/inode, arquivo estável, SHA-256 e executável, sem ler ambiente ou memória dos processos. Identidade do arquivo não demonstra equivalência ao source nem consumo algorítmico.

`ackermann_interface_contract.md` e `audit_ackermann_geometry.py` reproduzem a geometria estática calculada pelo plugin: L=6,816500073 m, W=2,598120190 m e raio traseiro 0,536 m. Resolver poses relativas/rotacionadas é necessário; a diferença simples entre eixos ignorava um offset da colisão dianteira direita.

Há conflito físico ainda aberto: `max_steer=0.6458` exige aproximadamente 0,721643/0,582648 rad nos joints, além de ±0,524. O bound virtual conservador calculado é 0,479957 rad, equivalente ideal a raio 13,094652 m; o raio configurado 11,6 m não cabe nesse bound. Nenhum desses números foi usado para retunar o veículo.

A inversa ideal, para velocidade não nula, é `u=atan2(L*omega, abs(v))`, considerando a inversão adicional do plugin em ré. O documento cobre zero assinado, quatro quadrantes, saturação conjunta, origem da base/eixo traseiro, possível caso de sinal no limite de ré MPPI 1.2.10, falhas e watchdog final. O futuro conversor precisa ficar depois do smoother e cobrir também o caminho direto do behavior_server. **Nenhum adaptador de atuação foi conectado.** O aviso do steering_wheel_joint ausente refere-se ao volante visual opcional; não explica a incompatibilidade de unidades.

## 4. Percepção e parâmetros aplicados

Não foi identificado produtor de `/external_area_pointcloud` nos fontes ou no grafo recebido. `virtual_wall_points` é outro tópico. A fonte configurada foi preservada: não remover obstáculos, criar nuvem vazia, remapear ou habilitar Virtual Lanes para mascarar a ausência. O diagnóstico registra publishers/subscribers e amostras quando existirem. O scan com 360 valores +inf pode indicar ausência de retornos; não demonstra detecção de obstáculo conhecido.

Os dumps CLI antigos (planner timeout 124; controller vazio com retorno zero) permanecem evidência. O novo coletor usa GetParameters/DescribeParameters explícitos, tipados, com lotes de até 16, quatro chamadas simultâneas no máximo, timeout de 2 s e até três tentativas. ListParameters é auxiliar. Em rclcpp 21.0.8, um nome não declarado pode esvaziar toda a resposta; por isso os obrigatórios têm prioridade e os opcionais são consultados individualmente. Resposta vazia não vira NOT_SET fabricado. Tentativas/respostas parciais sobrevivem a falhas de outros observadores.

Respostas observadas no deadline ou depois dele ficam `LATE_RESPONSE`, com payload preservado na tentativa e sem alimentar a validação efetiva. A janela RPC é encerrada antes da espera pelo observador C++. Smac/MPPI também exigem correspondência entre hashes instalados e mapeados; ausência ou divergência suspende a atribuição de consumidor ao pacote inspecionado.

O contrato cobre 128 nomes de Smac/MPPI, IDs, tipos, críticos, defaults e namespaces. Há parâmetros históricos sem consumidor identificado na versão 1.2.10, incluindo `GridBased.allow_primitive_interpolation`, `GridBased.analytic_expansion_max_cost`, `FollowPath.wz_min/reset_period` e campos de visualização no namespace incorreto. Os valores de inflação de ObstaclesCritic são obtidos da camada, não desses knobs locais. O YAML fica intacto; esses nomes não podem entrar cegamente no espaço do otimizador.

`plugin_parameters.json` separa **DECLARED**, **EFFECTIVE**, atribuição de consumidor no source e **ACTUALLY_USED=NOT_VERIFIED**. O snapshot de log antes do probe preserva sete padrões de evidência interna de configure reconhecidos no log recebido. Isso sustenta apenas os valores explicitamente impressos, não execução de trajetórias/algoritmos. Divergência de versão suspende a atribuição ao source inspecionado. A captura global pode ser PARTIAL por opcionais não declarados mesmo quando `required_collection_status=COMPLETE`; ler os dois campos e os checks por parâmetro.

## Entrega, teste e critérios de fechamento

Os arquivos completos, testes e logs offline estão nesta PR. **264 testes PASS**, auditoria estrutural PASS, sintaxe de 12 blocos Bash e quatro Python embutidos do runbook PASS. `evidence/second_review_summary.json` registra contagens, hashes e limites; o inventário de entrada preserva a identidade dos 115 arquivos. Testes usam fixtures e compilação do núcleo C++ sem ROS. Neste Work não houve Docker, compilação do adaptador ROS, launch, missão, Twist ou alteração no host. Esses resultados não substituem o novo build/probe.

| Pendência | Evidência necessária para fechar seu escopo |
|---|---|
| Variante carregada | IMAGE_VALIDATED, hashes instalados corretos e comando de launch com o novo mundo; comportamento observado compatível. |
| Conflitos TF | Seis arestas/pais URDF com RSP único; nenhum pai concorrente; odom TF, /odom e seis juntas preservados. |
| Frescor TF | Clock C++ avançando na mesma janela, GIDs identificados, stamps dinâmicos avançando e idades dentro da política; explicar perdas/backlog se observados. |
| Frames/lever arms | Headers gps_link/imu_link e TF fixos iguais ao modelo; comparações exatas de GNSS/odom/TF disponíveis, com residual e limites interpretados pelo revisor. |
| Localização aprovada | Além dos frames: política de covariância fundamentada, tempo interno navsat verificado e alinhamento geométrico/mapa/rota com referência adequada. Continua aberto. |
| Atuação aprovada | Identidade/semântica/geometria/limites reconciliados, conversão e watchdog testados, autoridade única; teste físico futuro explicitamente autorizado. Continua aberto. |
| Percepção | Produtor externo identificado ou alteração arquitetural explícita revisada; dados e transformação válidos, detecção de obstáculo conhecido comprovada em protocolo autorizado. Continua aberto. |
| Smac/MPPI efetivos | Get/Describe obrigatórios completos, tipos/valores corretos, versão/lib/plugin XML registrados; nomes sem consumidor excluídos da hipótese de otimização até revisão. Consumo em execução ainda não demonstrado. |
| Regressão Nav2 | Mesmas costmaps/BTs confirmadas e sete lifecycle nodes ACTIVE na nova imagem. |

Executar somente a seção **2.3** do runbook: build novo → IMAGE_VALIDATED → probe novo → ZIP → revisor independente. `OBSERVATIONS_COLLECTED` não aprova saúde integral. PR draft, `mission_ready=false`, Gates 0/1 NOT_YET_VERIFIED. As fontes primárias, versões e blobs inspecionados estão nos três contratos técnicos vinculados acima.
