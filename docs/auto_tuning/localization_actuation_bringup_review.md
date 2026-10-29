# Revisão de localização e atuação — probe b8922ae

**Gate 0 e Gate 1: NOT YET VERIFIED. Movimento permanece bloqueado.** Esta revisão usa evidência local fornecida pelo proprietário e inspeção de fontes; não executa ROS/Gazebo neste Work nem certifica o runtime histórico privado.

## Evidência e identidade

ZIP recebido: `nav2_phase_a_probe_evidence.zip`, SHA-256 `6a48d2c9e96d8ca9202763dcb5f16250591afbbc598937c40b0fe936f6972034`, 110 arquivos. Os caminhos abaixo são relativos à raiz desse ZIP.

- Commit registrado: `b8922aee9e792b357dc15263a256e11a08cdbfb1`.
- Imagem registrada: `sha256:8fef65a928d1e8cda20724dfb24404d3396755c3d545170b52b7e3d0acc3ac16`.
- `build/status.json`: `IMAGE_VALIDATED`; `probe/status.json`: `OBSERVATIONS_COLLECTED`. Esses estados não demonstram prontidão integral do Nav2.
- `build/installed-packages.tsv`: Gazebo `11.10.2+dfsg-1`, `ros-iron-gazebo-plugins` `3.7.0-3jammy.20241114.235922`, MPPI `1.2.10-1jammy.20241114.232841`, robot_localization `3.7.1-1jammy.20241114.232719`.
- `probe/bringup.log`: 1004 ocorrências de robot out of bounds, 101 de ausência de mapa na StaticLayer local, uma de `Steering wheel joint [steering_wheel_joint] not found.`. A falha BT é tratada separadamente pela configuração prospectiva de BTs.

As versões de pacote sustentam a correspondência com as fontes upstream citadas. Elas não são uma auditoria de equivalência binária ou prova de qualquer patch privado; a próxima coleta deve reter também caminhos e hashes das bibliotecas carregadas.

## TF: cadeia existe parcialmente, autoridade por aresta ainda não demonstrada

| Transformação | Contrato prospectivo | Evidência recebida e limite |
|---|---|---|
| `map → odom` | EKF global, `world_frame=map`, `publish_tf=true` | Há composição `map → base_footprint` em `tf_map_base.log`, mas não captura direta dessa aresta com identificação do publicador. |
| `odom → base_footprint` | Gazebo Ackermann; EKF local não publica TF | Log do plugin declara essa publicação e `tf_odom_base.log` contém amostras. Falta associar cada mensagem ao GID do emissor. |
| `base_footprint → base_link` | Joint fixo URDF, translação `(0,0,0.877)`, rotação identidade | Contrato estático inspecionado. O probe antigo não registra explicitamente a aresta. |
| `base_link → cloud` | Joint fixo URDF `(4.8525,0,0.21925)`, rotação identidade | `tf_base_cloud.log` é compatível: `(4.853,0,0.219)` arredondado, rotação identidade. Não aparece o yaw legacy de 90°. |
| `base_footprint → gps_link` | Cadeia fixa via base_link/cabin_link: `(3.8205,0,3.062)` | Concordância SDF/URDF por inspeção; falta observar TF com o header correto do sensor. |
| `base_footprint → imu_link` | Cadeia fixa via base_link/cabin_link: `(3.7205,0,2.595)` | Concordância SDF/URDF por inspeção; falta observar TF com o header correto do sensor. |

`tf_publishers.log` lista quatro endpoints em `/tf`: Ackermann, robot_state_publisher, EKF local e EKF global. **Quatro endpoints não provam quatro autoridades para uma mesma aresta.** Em robot_localization 3.7.1 o broadcaster é criado independentemente de `publish_tf`; a condição controla o envio. Portanto o endpoint do EKF local pode existir sem publicar transformações. O relatório deve associar mensagens efetivamente recebidas a `MessageInfo.publisher_gid`, mapear o GID ao nó e contar emissores por `(parent, child)` durante uma janela limitada.

`tf_static.log` contém apenas dois warnings de RCL e terminou com exit 124; não contém uma TFMessage. Isso não prova ausência de TF estático, pois `tf2_echo base_link cloud` conseguiu obter a transformação. A próxima coleta precisa de assinatura compatível com `/tf_static` (durabilidade transient local, confiabilidade reliable), preservando todas as mensagens latched e seus emissores. Timestamp zero de transformação estática é válido e não deve ser marcado como dado dinâmico vencido.

Há outro risco de autoridade fora da cadeia principal: `publish_wheel_tf=true` no plugin Ackermann publica transformações do modelo para links de rodas, enquanto robot_state_publisher também pode publicar os links correspondentes a partir de `/joint_states`. Capturar pais concorrentes para o mesmo child e publicadores distintos por aresta; não deduzir consistência apenas pela existência dos seis nomes de joint.

## GNSS e IMU: metadados de frame incompatíveis com a posição física

O sensor GPS ativo está dentro do link SDF `base_footprint`, com pose `(3.8205,0,3.062)` e sem `frame_name` no plugin. `SensorFrameID()` de gazebo_ros 3.7.0 usa o link pai quando esse campo não existe. A amostra observada em `gps.log` confirma `header.frame_id=base_footprint`. A posição geográfica é a do sensor deslocado; o header declara o centro do veículo. Isso faz a compensação de lever arm do navsat receber a identidade entre base e GPS, em vez da translação física conhecida.

O sensor IMU ativo também está no link SDF `base_footprint`, deslocado `(3.7205,0,2.595)`, com orientação identidade e sem `frame_name`. `imu_remapped.log` confirma o mesmo header. Como os eixos físicos e os eixos da base têm a mesma rotação estática, esse fato por si só não prova yaw errado em repouso. Ainda assim, o header mascara a localização da IMU e precisa de tratamento explícito antes de interpretar acelerações durante curvas.

Correção mínima **proposta** para um modelo prospectivo separado: declarar `frame_name=gps_link` e `frame_name=imu_link` usando as transformações já presentes no URDF, sem mover sensores, alterar datum, escala, geometria ou ruído. A remap GPS duplicada (`~/out:=fix` e `~/out:=data`) deve ser substituída por uma única saída no modelo prospectivo. O probe recebido mostra `/gps/fix` e `/imu` ativos; `/gps/data` e `/imu/data` não publicam. Não aplicar uma transformação adicional arbitrária para compensar o erro de header.

Essa proposta de metadados não foi aplicada ao SDF/URDF canônico por esta revisão. Enquanto não houver uma variante explícita e nova evidência, a prontidão geométrica deve permanecer bloqueada. O valor horizontal do lever arm GPS é 3.8205 m, material frente às tolerâncias de navegação; a simples concordância de `/odometry/gps` com o EKF global não valida a origem física porque ambos podem compartilhar o mesmo erro.

## Covariâncias: distinguir modelo de simulação de precisão conhecida

| Evidência | Observado | Interpretação e condição pendente |
|---|---|---|
| `gps.log` | Fix válido; `position_covariance_type=2`; nove zeros | GPS SDF não declara ruído. O plugin preenche a diagonal a partir das variâncias e marca DIAGONAL_KNOWN. Isso explica o valor, mas não justifica usar precisão infinita no estimador ou extrapolar para o sensor real. |
| `odometry_gps.log` | Covariância pose de 36 zeros | Consistente com entrada GPS zero e sua rotação; não demonstra incerteza calibrada. |
| `imu_remapped.log` orientação | Nove zeros | Pelo contrato sensor_msgs/Imu, significa covariância desconhecida. O plugin 3.7.0 tem TODO para covariância de orientação. Não equivale a yaw exato. |
| IMU velocidade angular | Diagonal `4e-8` | Compatível com desvio padrão SDF `2e-4` elevado ao quadrado. |
| IMU aceleração linear | Diagonal `0.000289` | Compatível com desvio padrão SDF `0.017` elevado ao quadrado. |
| `odometry_raw.log` | x/y `1e-5`, yaw `0.001`; eixos não modelados `1e12` | Defaults do plugin; são valores configurados, não estimativa empírica de erro. |

O novo diagnóstico deve registrar tipo, diagonal, finitude, simetria e condição nula/desconhecida. Não preencher números inventados para fazer o check passar. A política de fusão para covariâncias desconhecidas/noiseless precisa ser decidida e versionada antes de uma missão. O EKF global está em `two_d_mode=false` no perfil herdado; pequenas inclinações observadas merecem revisão, mas não autorizam retuning silencioso.

## Alinhamento, datum, `/fromLL` e tempo

As amostras são sequenciais, não sincronizadas. Yaw calculado das mensagens: `/odom` aproximadamente `-1.105942` rad, `/odometry/local` `-1.105954`, `/imu` `-1.105928`, `/odometry/global` `0.000088`. As três primeiras são compatíveis em repouso, mas o yaw global está no frame `map`; sua diferença para `odom` **não prova erro**, pois há uma rotação `map → odom`. Comparar quaternions após transformação ao mesmo frame e timestamp, nunca números crus de frames diferentes.

O mundo contém origem geográfica `(-22.617911667,-47.502177333)`, elevação 488 m e heading 180°. O caminhão nasce em `(8.416977,9.692661,0.38)` com yaw `5.176624799` rad. O datum preservado é `[-22.617921667,-47.502277333,2.035032145]`; o navsat registra convergência meridiana `0.0168052`. Esses referenciais são diferentes por configuração; não substituir um pelo outro para aproximar posições. A altitude GPS observada de aproximadamente 491.062 m inclui o deslocamento vertical do sensor; `zero_altitude=true` força z de saída a zero e impede usar essa saída como prova de alinhamento vertical.

`from_ll.log` contém somente o nome do tipo `robot_localization/srv/FromLL`. O comando antigo não enviou uma requisição nem inspecionou uma resposta. A presença do serviço não valida conversão. Consultas limitadas e sem movimento podem registrar:

1. Datum convertido para a origem esperada de `map`, com entrada/saída e parâmetros efetivos preservados.
2. Fix GNSS corrente convertido para `map` e comparado à posição TF de **gps_link**, no timestamp correspondente. `/odometry/gps`, após compensar lever arm, deve ser comparada a **base_footprint**. Misturar esses pontos esconderia o erro de origem.
3. Alguns pontos distintos da rota pública convertidos uma única vez para inspeção de escala/orientação, sem envio de action. A rota pública marcada SCENARIO 5_6 não pode ser associada automaticamente aos CSVs S1–S3.

O sender histórico aplica ainda `+4.8025 m` no eixo x de map. Essa translação está preservada, mas não equivale automaticamente a mover um ponto para a frente do caminhão quando o heading muda. A associação entre rota, ponto de referência do veículo, costmaps e métricas precisa ser revisada antes de habilitar esse sender.

`clock.log` contém uma amostra em 33.773289 s; os logs TF apresentam timestamps posteriores e progressivos. Isso é indício de tempo de simulação em evolução, mas a coleta antiga não mede diretamente sequência de `/clock`, taxas, idade ou defasagem entre mensagens. A próxima janela deve usar relógio de parede apenas para timeout e comparar stamps com o `/clock` recebido: avanço positivo, ausência de regressão, amostras múltiplas e idade limitada por tópico. Um warning inicial de tf2_echo enquanto sua assinatura ainda descobre frames não deve sozinho reprovar a sessão inteira; ausência persistente ou TF vencida deve reprovar.

## Ackermann: aviso do volante é não fatal; contrato de comando continua bloqueado

O aviso `steering_wheel_joint` refere-se ao joint opcional do volante visual. Em gazebo_plugins 3.7.0, sua ausência reduz a lista de sete para seis joints e a atualização visual `SetPosition()` só ocorre quando a lista tem sete. O plugin continua carregando as quatro rodas e os dois joints de direção. Se algum desses seis joints essenciais não existir, o código registra ERROR, destrói o nó e retorna; o log recebido, ao contrário, confirma assinatura de `/cmd_vel`, publicação de `/odom`, TF e reconhecimento dos seis joints pelo publicador de joint_states.

Portanto o aviso isolado não explica a falha de ativação do BT nem comprova falha da direção das rodas. Também não prova capacidade de atuação correta. Não adicionar um joint fictício ou suprimir o aviso para obter um log limpo.

O contrato de comando é um bloqueador material:

- Nav2 MPPI 1.2.10 integra `wz * dt` como yaw e entrega `wz` em Twist: taxa angular.
- O plugin Gazebo 3.7.0 copia `Twist.angular.z` para `target_rot_`, multiplica pelo sinal da velocidade comandada, limita por `max_steer`, aplica `tan()` e calcula os ângulos esquerdo/direito. É um comando de esterçamento intermediário, não uma taxa angular.
- O caminho público observado configura controller → `/cmd_vel_nav` → velocity_smoother → `/cmd_vel`; não contém conversor identificado. Os parâmetros ROS em formato XML legado sem atributo `type` são ignorados pelo plugin, segundo o log, mas os tópicos efetivos coincidem com os defaults. Isso deve ser documentado, não confundido com validação da interface.
- Wheelbase é calculada pelo plugin a partir dos centros das colisões das rodas dianteiras/traseiras selecionadas. As coordenadas estáticas dos eixos selecionados sugerem separação longitudinal 6.804 m, mas não bastam para congelar a wheelbase efetiva: conferir centros completos e convenções do modelo em execução. Não substituir por 5.950 m a partir de memória histórica, nem usar o equivalente do evaluator como calibração.

Antes de qualquer movimento, exigir contrato versionado do conversor, incluindo sinais em marcha à ré (o plugin já altera o sinal), limite de esterçamento, velocidade próxima de zero, inputs inválidos, saturação e expiração de comando. O plugin inspecionado não implementa timeout de comando próprio; o `velocity_timeout=1.0` do smoother observado não substitui a análise do caminho completo, especialmente quando esse nó está inativo. Nenhum Twist de teste foi enviado nesta revisão.

## Critério da próxima coleta

O relatório legível por máquina deve separar pelo menos: `collection_completed`, `lifecycle_all_active`, `effective_parameters_match`, `clock_advancing`, `sensor_freshness`, `tf_edges_observed`, `tf_unique_authority_observed`, `from_ll_response_valid`, `geometric_localization_validated` e `actuation_interface_validated`. Um endpoint ausente, amostra insuficiente ou GID não identificável deve produzir UNKNOWN/FAIL explícito, nunca PASS por falta de dados. Mesmo com primeiros checks aprovados, os dois últimos permanecem pendentes de revisão e mantêm `mission_authorized=false`.

Não alterar datum, modelo físico, YAMLs históricos, Humble ou ambientes existentes. Não habilitar Virtual Lanes para compensar problemas do núcleo; não enviar goals, missões ou comandos de atuação. A próxima evidência deverá voltar ao revisor independente.

## Fontes primárias inspecionadas

- [Gazebo Ackermann 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp): joint opcional/essenciais, linhas 220–294; geometria 329–352; atuação 513–555. Blob `8f536598f837acef619337f4b72b626e7021dfa8`.
- [SensorFrameID 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_ros/src/utils.cpp), linhas 52–60; blob `c1554a078462e67dcf332a9da7d24f03f4e613c1`.
- [GPS 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_gps_sensor.cpp), frame, covariâncias e posição; blob `b55d28341c13e131cf7acca6482d2c4026f0df01`.
- [IMU 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp), frame/covariâncias 100–118; blob `c2462a7be25c4640db1c93561ed500ff7d126879`.
- [sensor_msgs/Imu Iron](https://github.com/ros2/common_interfaces/blob/iron/sensor_msgs/msg/Imu.msg): contrato de covariância desconhecida; blob `8d8a6bdaf34a11150826d160162a54fa1cc323d4`.
- [robot_localization 3.7.1 navsat_transform](https://github.com/cra-ros-pkg/robot_localization/blob/3.7.1/src/navsat_transform.cpp): datum manual, fromLL, remoção do lever arm, covariância; blob `c879926641ee3eba1c77b1a0645fbe81410b6828`.
- [robot_localization 3.7.1 ros_filter](https://github.com/cra-ros-pkg/robot_localization/blob/3.7.1/src/ros_filter.cpp): criação do broadcaster e publicação condicional; blob `31821fe0870d80e809b27b63c29f876eae0d36ed`.
- [MPPI optimizer 1.2.10](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/src/optimizer.cpp), integração de yaw e Twist; [motion models](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp), restrição de raio. Blobs `86359395ef96081b9858ffc1fd1d61fa762a8d72` e `0c5671ea3e56f994c8257d19026c9dfecd8fb34d`.
