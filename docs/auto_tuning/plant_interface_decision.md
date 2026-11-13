# Decisão prospectiva da planta e interface — revisão 4

Decisão: conservar temporariamente Gazebo Classic 11.10.2, ROS 2 Iron e o plugin
`gazebo_ros_ackermann_drive` 3.7.0, acrescentando um adaptador ROS 2 pequeno.
Não desenvolver outro plugin de física. A escolha minimiza mudanças na planta
que já produziu evidências no ipt-sim-01; não demonstra superioridade física do
Classic. A aceitação depende dos ensaios delimitados no protocolo prospectivo.

Esta entrega prepara um candidato completo, isolamento do tópico final e um
adaptador de observação que recebe mensagens ROS e calcula comandos candidatos
sem publicá-los. Não conecta uma conversão ainda sem validação dinâmica à planta.
Os testes locais são estáticos/offline; a última evidência real recebida do host
continua sendo a21403f. Nem d17fcec nem este incremento foram executados no host
por este trabalho. Gates 0 e 1 permanecem NOT_YET_VERIFIED.

## Comparação causal

| Item | Classic com adaptador | Gazebo Sim Harmonic nativo |
|---|---|---|
| Twist | Converter referência de velocidade e yaw rate em esterçamento | Sistema AckermannSteering recebe velocidade e yaw rate |
| Controle das rodas | PID de torque existente, mesma força nas rodas traseiras | Comandos de velocidade das rodas e controle P do esterçamento |
| Odometria | Pose/velocidade mundial do modelo | Integração das rodas e média dos ângulos de esterçamento |
| Expiração do comando | Ausente no plugin inspecionado | Também ausente na versão 8.15.0 inspecionada |
| Sensores/ROS | Mantém plugins e interfaces observados | Substitui plugins ROS diretos por sistemas e ros_gz_bridge |
| Física e SDF | Mantém ODE, contato, massas e geometria | Novo backend; DART padrão, parâmetros de contato precisam ser revalidados |
| Docker | Mesmas dependências, pequenas adições de arquivos | Imagem distinta, Gazebo e bridge compatíveis, novos smoke/probes |
| Validação adicional | Referências, conversão, limites, parada e dinâmica | Os itens anteriores mais sensores, física, TF, odometria, renderização e datum |

Harmonic tem binários para Jammy e Noble. Para Iron, o README da série Iron
documenta Harmonic de fonte; a documentação de instalação também descreve
pacotes OSRF não oficiais `ros-iron-ros-gzharmonic`, conflitantes com
`ros-iron-ros-gz*`. Isso não é uma troca transparente no Docker atual. Se migrar
vier a ser necessário, usar imagem separada, nunca instalar no host ou misturar
dependências no ambiente já validado. Iron/Classic encerraram suporte: conservar
IDs, inventário, fontes e imagens é necessário, sem afirmar que um digest da
base fixa todo o APT.

O modelo contém sensores ray, câmera, GNSS e IMU com plugins Classic. O guia
oficial de migração substitui ray por gpu_lidar e integração direta por bridge;
isso amplia a validação da percepção/renderização. A compensação histórica de
heading/datum não deve ser removida implicitamente. O SDF carregar sem erro não
prova equivalência de dinâmica, sensores ou TF.

## Semântica efetiva e correção

No Classic 3.7.0, o callback armazena `linear.x` e `angular.z`. O update usa
`angular.z * sign(linear.x)` como ângulo virtual, limita-o por `max_steer` e
calcula os dois ângulos das rodas. O PID longitudinal lê **somente a roda
traseira direita**, compara sua velocidade angular ao alvo dividido pelo raio
e aplica o mesmo torque às duas rodas traseiras. Não há prazo de validade.

Para a referência planar no centro do eixo traseiro, sob rolamento ideal:

\[
\delta=\arctan(L\omega/v_R),\quad
v_{plugin}=v_R-\omega y_{RR},\quad
u_{plugin}=\operatorname{sign}(v_{plugin})\delta.
\]

`y_RR=-1.299 m` vem da geometria traseira, não da bitola dianteira por conveniência.
Em ré, o sinal é compensado exatamente uma vez. A conversão separa velocidade do
eixo, alvo da roda direita, yaw rate e esterçamento; verifica limites para ambos.
Zero com yaw não nulo é irrealizável e é rejeitado. NaN/Inf, curvatura impossível,
inversão imprópria e dados vencidos não são aceitos. A política shadow rejeita
excesso de velocidade; o núcleo também testa redução conjunta que preserva a
curvatura. Nenhuma dessas equações comprova ausência de escorregamento, resposta
PID, frenagem ou sinal físico das juntas.

No Gazebo Sim 8.15.0, o sistema calcula `R=v/omega`, impõe
`Rmin=L/sin(steering_limit)` e determina ângulos/velocidades diferenciais. Não se
deve copiar `max_steer` do Classic para `steering_limit`: as fórmulas diferem.
Os mesmos números min/max de velocidade, aceleração e jerk são atribuídos aos
limitadores linear e angular nessa versão; `min_velocity=0` também impediria
yaw negativo. Com v=0 e yaw suficiente, ele esterça sem tração. Seu callback
também apenas conserva o último comando. Migrar não dispensa supervisão.

## Geometria, referência e velocidades

| Grandeza | Histórico preservado | Candidato explícito |
|---|---|---|
| Distância calculada pelo plugin | L=6.816500073351341 m | Igual; medir no modelo carregado antes da aceitação dinâmica |
| Limite das juntas dianteiras | ±0.524 rad | Igual |
| Ângulo virtual permitido pelas duas juntas | Aproximadamente 0.479956863 rad | max_steer derivado de atan(L/13.10), abaixo desse limite |
| Raio de Smac/MPPI | 11.6 m | 13.10 m, arredondamento superior do limite ideal 13.094652364 m |
| Referência Nav2 | base_link/base_footprint | rear_axle_nav, filho fixo em x=-3.502 m de base_footprint |
| Footprint | Polígono histórico | Mesmo polígono físico, coordenadas x acrescidas de 3.502 m |
| Critério do objetivo | FrontCabinGoalChecker, offset implícito 5 m | SimpleGoalChecker já configurado, objetivo no eixo traseiro |
| Velocidade máxima MPPI/smoother | 25 / 8.33 m/s | 50/9 m/s (20 km/h), depois 125/18 m/s (25 km/h) |
| Ré na navegação inicial | MPPI -2.25 m/s | Desabilitada no perfil inicial; conversão em ré testada offline |
| Yaw máximo | MPPI 0.4; smoother 0.72 rad/s | ±0.4 rad/s nos dois envelopes |

13.10 m é compatibilidade numérica estática, **não margem dinâmica comprovada**.
É raio do eixo traseiro, não círculo externo varrido nem raio da cabine. O eixo
dianteiro não perfeitamente retangular, rodas intermediárias fixas e atrito
permanecem preservados e precisam ser observados nos ensaios; não se calibram
massas/geometria para tornar um check verde.

20/25 km/h são tetos experimentais pedidos, não velocidades para o primeiro
ensaio de calibração, nem velocidade constante em curva. Com yaw limitado a
0.4 rad/s, esses tetos implicam raios de pelo menos 13.889/17.361 m. Não há
evidência para reinterpretar 25.0 histórico como 2.5. A configuração histórica
do smoother e os parâmetros de aceleração não constituem limites físicos
medidos; sua interação com a conversão e a parada integra o ensaio posterior.

## Topologia e falhas

No candidato, o plugin escuta somente `/actuation/steering_command`.
O probe exige zero publishers nesse tópico e o subscriber Ackermann esperado.
Nav2 continua produzindo yaw rate no seu grafo; a observação o recebe sem ligar
a saída. Publishers históricos do behavior_server em `/cmd_vel` ficam visíveis
e não podem contornar o isolamento da planta.

Antes de conectar saída, o caminho autorizado deve ter fonte identificada e
arbitragem explícita antes do smoother; o adaptador será a única autoridade no
tópico final. Prazo monotônico de comando, odometria/clock, mudança de GID,
não finitos, ré e saturação devem produzir estado inibido. Um timer no próprio
adaptador não resolve sua morte. O supervisor externo deve encerrar apenas o
simulador/container pertencente ao ensaio se a autoridade final morrer ou
perder sua lease; falha do supervisor também deve encerrar sua sessão. Esse
encerramento é uma interrupção de simulação, não prova de frenagem física.
Esse caminho de publicação/supervisão ainda não está habilitado nesta entrega.

A decisão tem condição de saída: se os ensaios delimitados mostrarem limitação
estrutural do plugin que exija reescrever atuação/odometria, reavaliar Harmonic
antes de criar plugin próprio. Warnings cosméticos não prorrogam essa decisão.

## Fontes oficiais inspecionadas

- [Classic 3.7.0, fonte Ackermann](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp), blob `8f536598f837acef619337f4b72b626e7021dfa8`; update/callback/odom, linhas 507–597.
- [Gazebo Sim 8.15.0, fonte imutável](https://github.com/gazebosim/gz-sim/blob/446a44335a45b704b4d36dabcc5508ee34eeb3d8/src/systems/ackermann_steering/AckermannSteering.cc), blob `f1398c6c9588d3779c59ec6effcca19103f7b494`; limites 334–378, odom 727–744, conversão 822–904.
- [Parâmetros oficiais AckermannSteering 8](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html).
- [Migração de pacotes ROS 2 Classic](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/).
- [Instalação ROS/Gazebo e combinações](https://gazebosim.org/docs/harmonic/ros_installation/).
- [ros_gz Iron, README fixado](https://github.com/gazebosim/ros_gz/blob/14e4465305ac96beb4c93c506552783fbf6a46a0/README.md).
- [Física do Gazebo Sim 8](https://gazebosim.org/api/sim/8/physics.html).

O código de uma tag documenta semântica de fonte; identidade do .so instalado e
mapeado continua no probe e não equivale automaticamente à reprodução binária
desse fonte. Não foram copiadas nem modificadas bibliotecas instaladas.

## Evidência desta entrega

416 testes offline PASS; auditoria estrutural PASS; sintaxe de 29 blocos Bash e
11 blocos Python embutidos verificada. Oito fontes canônicas/da variante anterior
mantiveram seus hashes. A revisão independente detectou e corrigiu a ausência
de publisher GID no MessageInfo do rclpy Iron: timestamps/sequências são
preservados, indisponibilidade não é confundida com conflito de autoridade.

Resultados e hashes em `evidence/plant_review_summary.json`, log integral em
`evidence/plant_review_tests.log`, verificações em `evidence/plant_review_checks.log`
e exemplos matemáticos em `evidence/plant_review_conversion_examples.json`.
Esses testes não executaram Docker, ROS, GUI ou movimento e não concedem PASS
dinâmico à planta. A próxima etapa completa está na seção 2.5 do runbook.
