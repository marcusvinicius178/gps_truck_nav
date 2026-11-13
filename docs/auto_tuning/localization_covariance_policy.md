# Variante de modelo e política de localização — segunda revisão

**Escopo: diagnóstico prospectivo, sem movimento. Gate 0 e Gate 1: NOT_YET_VERIFIED.** A correção dos headers e da autoridade TF não aprova a localização geométrica nem a fusão de sensores.

## Evidência recebida e causa

O ZIP `nav2_phase_a_978ab23_review_evidence.zip`, SHA-256 `984c670c9da54488992c5ff21adb48d374542e8dfcefaca61e4ac591ddee1da7`, contém o probe do commit `978ab2328c6cb378d6eea016027192a6cf1dbadf`. O relatório independente e os arquivos atribuídos a esta análise foram lidos integralmente. Os dados confirmam sensores recentes, seis conflitos de pai em TF e headers GPS/IMU `base_footprint` apesar das posições físicas deslocadas. As costmaps e as duas BTs já foram verificadas no host e não são modificadas nesta variante.

O modelo do mundo `ctvi.world` é incluído por `model://arocs_truck`; não é gerado pelo robot_state_publisher. O Gazebo carrega `models/arocs_truck/model.sdf`, enquanto robot_state_publisher recebe `urdf/arocs_truck.urdf`. Em gazebo_plugins 3.7.0, `publish_wheel_tf` controla uma chamada independente de `publish_odom_tf` e da mensagem `/odom`. As seis transformações publicadas por esse ramo usam a base como pai, concorrendo com a hierarquia articulada URDF. `/joint_states` vem de outro plugin, `truck_joint_state`; não depende desse ramo TF.

Os sensores foram agregados ao link SDF `base_footprint` pela conversão de joints fixos, preservando suas poses. Sem `<frame_name>`, `gazebo_ros::SensorFrameID()` devolve o nome desse link pai. O header, portanto, descreve a origem errada mesmo que a posição física do sensor esteja correta. `NavSatFix.header.frame_id` deve identificar a antena relativa ao veículo, não o elipsoide nem a origem da base.

## Alteração mínima, explícita e completa

| Arquivo novo | Contrato |
|---|---|
| `config/auto_tuning/models/arocs_truck_tf_sensors/model.sdf` | Cópia completa do SDF, com exatamente três alterações: `publish_wheel_tf` 1→0; inserção de `frame_name=gps_link` no GPS; inserção de `frame_name=imu_link` na IMU. |
| `config/auto_tuning/models/arocs_truck_tf_sensors/model.config` | Bytes idênticos ao model.config canônico. |
| `config/auto_tuning/worlds/ctvi_tf_sensors.world` | Cópia completa de CTVI, com uma única alteração: URI do caminhão para `model://arocs_truck_tf_sensors`. |
| `config/auto_tuning/tf_sensor_model_variant.json` | Identidade da variante, hashes canônicos/novos, alterações permitidas, seis arestas URDF e seis joints esperados. |
| `tools/auto_tuning/validate_model_variant.py` | Verifica os hashes, a equivalência exata dos demais bytes, os lever arms fixos SDF/URDF e a resolução das meshes canônicas; não inicializa ROS/Gazebo. |

O nome interno do modelo continua `arocs_truck`. As URIs `model://arocs_truck/meshes/...` continuam apontando para os assets canônicos instalados; as meshes não são duplicadas. O novo diretório de modelos é acrescentado ao caminho da imagem isolada, e o probe seleciona explicitamente o novo mundo. Os launch files existentes já preservam o `GAZEBO_MODEL_PATH` recebido. Nenhum arquivo canônico é sobrescrito, nem há mudança em datum, spawn, geometria, massas, inércias, limites, ruído, remaps ou frequências. A remap GPS duplicada permanece registrada como legado; não é necessário alterá-la para resolver os três pontos desta variante.

O verificador aceita `--repo CAMINHO` para fontes ou `--installed-share CAMINHO --config CAMINHO` para a imagem. Sucesso produz `MODEL_VARIANT_VALIDATED` com `scope=STATIC_ASSET_AND_METADATA_CONTRACT_ONLY`, `runtime_verified=false`, `mission_ready=false`. Treze regressões exercitam preservação, alterações indevidas de geometria/datum/odom, drift dos pins, meshes ausentes/LFS e relatórios de falha; isso não substitui o próximo probe.

## Lever arms e comparação sincronizada

Os dois caminhos fixos URDF têm rotação identidade em relação à base:

| Header da variante | Posição em `base_footprint` (m) | Composição URDF |
|---|---|---|
| `gps_link` | `(3.8205, 0, 3.062)` | base_footprint → base_link → cabin_link → gps_link |
| `imu_link` | `(3.7205, 0, 2.595)` | base_footprint → base_link → cabin_link → imu_link |

O frame explícito não move o sensor nem muda seus eixos. Na IMU, a rotação fixa identidade explica por que o header anterior não necessariamente produzia erro de yaw em repouso; o deslocamento continua relevante para acelerações em movimento. O perfil atual nem sequer funde acelerações lineares da IMU. Não tratar esse fato como validação de aceleração no centro do veículo.

Para cada fix GNSS selecionado, registrar seu stamp inteiro `(sec,nanosec)`, entrada/saída de `/fromLL`, `/odometry/gps` com o mesmo stamp e lookup TF solicitado naquele stamp. Guardar também os tempos de requisição/resposta/recepção. Ausência de TF exato, resposta atrasada ou amostra sem correspondência deve aparecer como indisponível, sem substituir silenciosamente por `latest` ou pelo vizinho mais próximo.

Em notação de vetores, com `r` o lever arm em base e `R(t)` a rotação completa de base para map:

- Antena: `p_gps_map(t) = p_base_map(t) + R(t) r`.
- Compensação esperada: `p_base_from_gps(t) = fromLL(fix(t)) − R(t) r`.
- Comparar `fromLL(fix(t))` com `map→gps_link(t)` e `/odometry/gps(t)` com `map→base_footprint(t)`, nas componentes horizontais. Comparar também a compensação esperada com a saída navsat para localizar o efeito do lever arm.

Usar a rotação 3D completa. No probe recebido, o pitch global é aproximadamente 0,02016 rad; a altura da antena contribui para o deslocamento horizontal, de modo que 3,875 m não deve ser comparado apenas com 3,8205 m longitudinal. `zero_altitude=true` zera a saída vertical do navsat; não elimina o efeito de roll/pitch na projeção horizontal do lever arm nem valida alinhamento vertical.

O residual anterior de aproximadamente 3,875 m misturava fix em 125,000832 s com TF em 131,506848 s. Ele sustenta a investigação do header, mas não é uma medição sincronizada de erro. Um residual pequeno após a correção também não constitui validação independente quando estimador e TF compartilham os mesmos sensores. Precisão experimental exige referência independente, protocolo e limites definidos antes da execução. Nenhum novo limite geométrico de aceitação foi inventado para esta revisão.

## Política de covariâncias

**Política desta etapa:** preservar todas as mensagens/configurações; classificar explicitamente o significado das matrizes; manter a aprovação de fusão e qualquer missão bloqueadas quando a incerteza não está fundamentada. Não preencher diagonais por conveniência, não reclassificar um zero como precisão física demonstrada e não usar o floor numérico do filtro como calibração.

| Entrada observada | Origem e significado | Tratamento obrigatório |
|---|---|---|
| GPS: nove zeros, tipo DIAGONAL_KNOWN | O plugin 3.7.0 lê variâncias de ruído do sensor; a ausência de modelo de ruído dá zero, mas o plugin sempre atribui esse tipo. Compatível com GPS idealizado da simulação. | Preservar zero e tipo como evidência. Manter `BLOCKED_REVIEW` para a política de fusão/representatividade. Não extrapolar para Trimble ou outro receptor real. |
| IMU orientação: nove zeros | O plugin tem TODO para covariância de orientação. Pelo contrato de sensor_msgs/Imu, zero significa incerteza desconhecida. | Manter `BLOCKED_REVIEW`. Não trocar por −1: −1 significa ausência da estimativa, e a orientação existe. |
| IMU velocidade angular: diagonal `4e-8` | `0.0002²`, a partir do stddev SDF. | Verificar finitude, simetria e PSD, preservando o valor. Esse teste não calibra bias, correlação ou erro absoluto. |
| IMU aceleração: diagonal `0.000289` | `0.017²`, a partir do stddev SDF. | Mesma distinção entre consistência numérica e modelo físico validado. |
| `/odometry/gps`: covariância nula | navsat copia e rotaciona a covariância GPS; rotação de zero continua zero. | Preservar e manter `BLOCKED_REVIEW`. Concordância com o EKF que usa essa saída não prova sua precisão. |

Em robot_localization 3.7.1, o EKF eleva diagonais menores que `1e-9` para `1e-9` durante a correção. É uma proteção contra instabilidade da álgebra de Kalman; não estima o ruído desconhecido. O perfil atual funde yaw da IMU e posição GPS, portanto essa proteção pode conferir confiança extrema a entradas cuja incerteza não foi calibrada. Nenhuma máscara de fusão foi alterada silenciosamente aqui.

Antes de usar essas entradas em uma missão/experimento, a política versionada deve distinguir claramente sensor idealizado de referência e medição destinada à fusão. Para fusão representativa, obter variâncias/bias/correlações de dados ou especificação aplicável, registrar a procedência e implementar o modelo de ruído coerente na variante prospectiva. Quando não houver incerteza utilizável, excluir explicitamente a componente da fusão e validar uma fonte alternativa observável; não fabricar uma matriz nem apenas renomear o check para PASS. Uma experiência deliberadamente ideal/noiseless exige decisão metodológica própria e não demonstra robustez a sensores reais. Essas escolhas futuras não estão autorizadas por um `IMAGE_VALIDATED` ou por esta política diagnóstica.

## Risco temporal no source de robot_localization 3.7.1

A inspeção de `navsat_transform.cpp` 3.7.1 encontrou em `prepareGpsOdometry()` a construção de `rclcpp::Time` a partir de `sec + nanosec/1e9`, como um único `double`. O header de rclcpp 21.0.8 define o construtor de um argumento em **nanosegundos inteiros**, não segundos. Para essa combinação de fontes, um stamp de 125,000832 s acaba convertido para 125 ns. `lookupTransformSafe()` tenta o instante solicitado e, em falha, usa `TimePointZero`; nesse source os warnings correspondentes estão comentados. Assim, uma compensação aparentemente válida pode usar orientação recente em vez da orientação do fix.

Isso é uma conclusão da combinação de fontes inspecionadas, não prova de que o pacote binário instalado contém exatamente esse trecho. O ZIP identifica `ros-iron-robot-localization=3.7.1-1jammy.20241114.232719` e `ros-iron-rclcpp=21.0.8-1jammy.20241114.221434`; versões e hashes identificam o artefato, mas não estabelecem equivalência fonte-binário sem verificar o pacote/source correspondente. O hash da biblioteca Ackermann registrado no ZIP tampouco identifica a biblioteca navsat.

Nenhum arquivo em `/opt/ros/iron` foi editado. A coleta deve reter versão/identidade do artefato navsat e medir os residuais sincronizados; não declarar `ACTUALLY_USED` para compensação temporal interna só porque GetParameters e `/fromLL` responderam. Se o trecho for confirmado no binário/source do pacote, a correção futura deve ser um overlay isolado, com construção explícita a partir da mensagem de tempo e regressão de extrapolação/fallback, antes de movimento. A correção de headers feita agora não resolve por si só esse risco.

## Critérios verificáveis para a próxima revisão

1. Manifest e variante validados na nova imagem, com world/URI e hashes usados preservados nos logs; modelo canônico e quatro fontes protegidas intactos.
2. Todas as seis arestas de roda/esterçamento do manifest presentes com pais URDF e emissor robot_state_publisher; nenhum conflito de pai e nenhuma aresta de roda/esterçamento emitida por Ackermann. Ausência de todas as rodas não é resolução do conflito.
3. `/joint_states` continua publicando os seis joints configurados, com valores finitos e stamps avançando; `/odom` e `odom→base_footprint` permanecem disponíveis pelo Ackermann. Clock/TF/sensores têm janelas e referências temporais explícitas.
4. Mensagens GPS/IMU usam `gps_link`/`imu_link`, e seus TF fixos correspondem às poses SDF/URDF; confirmar também ausência de erros de transformação do navsat.
5. Comparações GNSS/TF/odometria preservam o mesmo stamp, pontos físicos e rotação completa. Dados insuficientes e dependência de fallback interno permanecem explícitos.
6. As três covariâncias nulas/desconhecidas permanecem pendentes de política de fusão, sem valores inventados. Mesmo que os critérios de frame/TF passem, esta revisão não autoriza missão, Gate 0, Gate 1 ou otimização.

## Fontes primárias inspecionadas

- [gazebo_ros_ackermann_drive.cpp 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp), flags e ramos independentes de publicação; blob `8f536598f837acef619337f4b72b626e7021dfa8`.
- [gazebo_ros_joint_state_publisher.cpp 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_joint_state_publisher.cpp), plugin independente de JointState; blob `1d93eb16701d2fffa120ae5c1d965aa58bdfe166`.
- [SensorFrameID/NoiseVariance 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_ros/src/utils.cpp), fallback ao link pai; blob `c1554a078462e67dcf332a9da7d24f03f4e613c1`.
- [GPS 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_gps_sensor.cpp), blob `b55d28341c13e131cf7acca6482d2c4026f0df01`; [IMU 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp), blob `c2462a7be25c4640db1c93561ed500ff7d126879`.
- [sensor_msgs/Imu Iron](https://github.com/ros2/common_interfaces/blob/iron/sensor_msgs/msg/Imu.msg), blob `8d8a6bdaf34a11150826d160162a54fa1cc323d4`; [NavSatFix Iron](https://github.com/ros2/common_interfaces/blob/iron/sensor_msgs/msg/NavSatFix.msg), blob `3c4e9770a5dc0af6277ad53d3c2bfb1764007539`.
- [navsat_transform.cpp 3.7.1](https://github.com/cra-ros-pkg/robot_localization/blob/3.7.1/src/navsat_transform.cpp), conversão/lever arm, linhas 615–657 e 846–899; blob `c879926641ee3eba1c77b1a0645fbe81410b6828`.
- [ekf.cpp 3.7.1](https://github.com/cra-ros-pkg/robot_localization/blob/3.7.1/src/ekf.cpp), floor de estabilidade, linhas 139–152; blob `601e188c85fcdc63ac9d9e49dbb8782a96d8b0a6`.
- [rclcpp time.hpp 21.0.8](https://github.com/ros2/rclcpp/blob/21.0.8/rclcpp/include/rclcpp/time.hpp), construtores em segundos/nanos explícitos e nanos de um argumento; blob `15533f39ef64229029e32c8176f48d8d42a94828`.
- [ros_filter_utilities.cpp 3.7.1](https://github.com/cra-ros-pkg/robot_localization/blob/3.7.1/src/ros_filter_utilities.cpp), fallback temporal; blob `1c547a11a9de55258db457b06b28cb200806e7c5`.
