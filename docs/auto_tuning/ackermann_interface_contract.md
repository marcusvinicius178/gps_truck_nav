# Interface Ackermann — contrato prospectivo, revisões 2–4

**Atualização vigente — revisão 4:** a decisão e a variante completa estão em
`plant_interface_decision.md`. O histórico abaixo permanece rastreável, mas a
próxima coleta usa o candidato Classic explícito: raio13.10 m derivado das
juntas, frame traseiro/footprint coerentes, perfis20/25 km/h e tópico final
isolado. O adaptador ROS receive-only `ackermann_adapter.py` calcula sem publicar.
Foi identificada uma correção adicional à hipótese longitudinal anterior:
o PID do plugin mede a roda traseira direita, portanto o alvo ideal é
`v_plugin=v_eixo−omega*y_direita`, com `y_direita=−1.299 m`. A fórmula de
esterçamento abaixo continua válida quando os sinais das velocidades coincidem;
a velocidade linear não deve ser copiada como se o feedback fosse no centro.
`actuation_contract.py` preserva o núcleo anterior; `convert_classic` acrescenta
essa transformação e limites independentes. Hipótese de rolamento, dinâmica e
parada permanecem pendentes. Fonte upstream não foi modificada. Nesta revisão,
o limite SDF prospectivo é alterado explicitamente, sem alterar juntas/geometria
ou arquivos históricos. Os parágrafos seguintes descrevem os achados originais.

**Movimento bloqueado. Gate 0 e Gate 1: NOT_YET_VERIFIED.** Este documento deriva a interface e explicita o que falta validar. O auditor associado só lê XML e calcula geometria/equações; não contém ROS, publisher, subscriber, goal ou comando de atuação. Nenhum conversor foi conectado ao bringup e nenhum parâmetro físico foi substituído.

**Atualização de 03/11/2026, terceira revisão:** o probe a21403f confirmou TF/headers e leitura efetiva de MPPI `vx_max=25.0`, `wz_max=0.4`, modelo Ackermann; o smoother limita ±8,33 m/s e ±0,72 rad/s. O grafo mostra quatro publishers do behavior_server que contornam o smoother. Não reinterpretar 25,0 como 2,5. Os conflitos geométricos abaixo permanecem estáticos e pendentes. Foi acrescentado `actuation_contract.py`, núcleo exclusivamente offline com regressões de conversão/limites/falhas; ele não é ligado à planta nem implementa um watchdog operacional. O protocolo completo e as condições futuras de autorização estão em `actuation_validation_protocol.md`.

## 1. Evidência instalada e limite de identidade

Fonte recebida: `nav2_phase_a_978ab23_review_evidence.zip`, SHA-256 `984c670c9da54488992c5ff21adb48d374542e8dfcefaca61e4ac591ddee1da7`, commit `978ab2328c6cb378d6eea016027192a6cf1dbadf`, imagem `sha256:39d492f24ddd66639f58c527e6dd81cde3682983fc4e856de41a1ccfb0b5d09c`.

`build/installed-packages.tsv`, lido integralmente, identifica:

| Componente | Versão registrada |
|---|---|
| Gazebo Classic | `11.10.2+dfsg-1` |
| `ros-iron-gazebo-plugins` | `3.7.0-3jammy.20241114.235922` |
| `ros-iron-nav2-mppi-controller` | `1.2.10-1jammy.20241114.232841` |
| `ros-iron-nav2-velocity-smoother` | `1.2.10-1jammy.20241114.230159` |

`readiness_observations.json` identifica o arquivo instalado `/opt/ros/iron/lib/libgazebo_ros_ackermann_drive.so`, SHA-256 `db9ea2d3c915b7f88c84295a922113225524526daf5cab70db15950e6abe9711`. Esse hash é evidência de identidade do arquivo lido, não demonstra sozinho que esse mesmo inode foi carregado por gzserver nem que o binário é idêntico ao código-fonte upstream. A nova coleta deve correlacionar `/proc/<gzserver>/maps`, caminho resolvido, hash do arquivo e versão de pacote; uma divergência é uma nova pendência, nunca uma justificativa para ignorar a identidade.

O log confirma assinatura `/cmd_vel`, publicação `/odom`, TF `odom→base_footprint` e reconhecimento dos seis joints essenciais. O joint `steering_wheel_joint` é opcional/visual: upstream reduz a lista para seis quando ele falta e só executa sua atualização visual com sete joints. O aviso não explica a falha de semântica de comando. Os parâmetros XML legados `cmd_vel_topic/odom_topic/distance_topic`, sem `type`, foram ignorados; os tópicos ativos coincidem com os defaults. Não confundir essas tags com prova de configuração aplicada.

## 2. Semântica e caminho de comando

MPPI 1.2.10 integra `wz × model_dt` no yaw e entrega `wz` em `Twist.angular.z`: **taxa angular, rad/s**. `AckermannConstraints.min_turning_r` restringe curvatura; não converte para ângulo de roda.

No plugin Gazebo 3.7.0, `OnCmdVel` copia `linear.x` para velocidade alvo e `angular.z` para `target_rot_`. A atualização usa:

\[
v_p = \operatorname{clamp}(v,-v_{max},v_{max}),\qquad
\delta = \operatorname{clamp}(u\,\operatorname{copysign}(1,v),-\delta_{max},\delta_{max})
\]

\[
\alpha_L = \operatorname{atan2}\!\left(\tan\delta,1-\frac{W}{2L}\tan\delta\right),\qquad
\alpha_R = \operatorname{atan2}\!\left(\tan\delta,1+\frac{W}{2L}\tan\delta\right)
\]

Aqui `u` é o campo recebido `angular.z`; `δ` é o ângulo virtual central após a inversão de sinal em ré; `αL/αR` são alvos dos joints esquerdo/direito. O plugin usa PID e forças para alcançar esses alvos, logo a equação do alvo não demonstra resposta física, aderência, velocidade real ou erro de rastreamento.

O launch atual liga controller → `/cmd_vel_nav` → velocity_smoother → `/cmd_vel` → Gazebo. O behavior_server também pode publicar diretamente em `/cmd_vel`; suas ações de movimento não são chamadas pelas duas BTs prospectivas, mas os servidores continuam existentes. Um futuro adaptador precisa de saída dedicada exclusiva para Gazebo e roteamento/arbitragem de todos os produtores de movimento, sem deixar um caminho que contorne a conversão. A nova observação do grafo deve registrar publishers e subscribers desses tópicos; não enviar mensagens para descobri-los.

O smoother observado opera a 20 Hz, OPEN_LOOP, `scale_velocities=false`, limites lineares ±8.33 m/s, angulares ±0.72 rad/s e timeout 1.0 s. Ele limita/suaviza componentes Twist e não altera unidades. Portanto a conversão deve ocorrer **depois** dele, enquanto sua entrada ainda é `(v,ω)`; não alimentar um smoother configurado em rad/s com ângulo de esterçamento. Os limites declarados no YAML MPPI (`vx_max=25`, `vx_min=-2.25`, `wz_max=0.4`) e `max_speed=20` do SDF não representam por si os valores consumidos; a leitura estruturada ampliada deve confirmar os primeiros e a identidade do modelo deve sustentar os segundos.

## 3. Geometria: cálculo estático reproduzível, validação runtime pendente

O plugin calcula `L` como a distância euclidiana entre médias dos centros das colisões dianteiras e traseiras selecionadas. Calcula `W` como a distância entre as colisões dianteiras e extrai o raio da primeira colisão da roda traseira direita. Não lê uma constante wheelbase=5.950 do veículo real nem usa simplesmente a diferença dos x dos joints de direção.

O auditor `tools/auto_tuning/audit_ackermann_geometry.py` resolve `relative_to`, rotação RPY e pose da colisão no modelo. Recusa frames desconhecidos, ciclos, modelos aninhados/includes, geometrias não suportadas, múltiplas colisões, eixos incompatíveis e entradas não finitas em vez de aproximá-los silenciosamente. O modelo canônico tem SHA-256 `b8f8fc8b20a129694eed58bfd439c770e112a70fcf0e6da37bd9595f7e7ca8f2`.

| Centro de colisão inicial, frame do modelo | x (m) | y (m) | z (m) |
|---|---:|---:|---:|
| `first_wheel_left_link` | 3.302 | 1.299 | 0.535 |
| `first_wheel_right_link` | 3.326999999999831 | −1.2989999081698724 | 0.535 |
| `fourth_wheel_left_link` | −3.502 | 1.299 | 0.536 |
| `fourth_wheel_right_link` | −3.502 | −1.299 | 0.536 |

O joint da roda dianteira direita tem translação local `(0,−0.025,0)` em um frame girado aproximadamente 90°; isso acrescenta aproximadamente 0.025 m em x. Resultam **L=6.816500073351341 m**, **W=2.598120190224205 m**, raio traseiro **0.536 m**. O valor 6.804 m sugerido pela diferença longitudinal dos eixos ignorava esse offset. A revisão não substitui nem 5.950 nem 6.804 por uma nova calibração: os valores acima são somente a reprodução estática do algoritmo do plugin e da árvore atual.

Há incompatibilidade explícita de limites. O SDF configura `max_steer=0.6458`, mas cada joint de direção permite apenas ±0.524 rad. Com as equações upstream, `+0.6458` pede aproximadamente **0.721643 rad à esquerda e 0.582648 rad à direita**, ambos além dos limites. O mesmo problema ocorre com sinal negativo. Usando o limite comum simétrico `a=0.524`, um bound virtual conservador é:

\[
\delta_{joint}=\arctan\left(\frac{\tan a}{1+\frac{W}{2L}\tan a}\right)
\approx0.479956863\;rad
\]

Nesse modelo ideal, `L/tan(δjoint)≈13.094652364 m`. O raio 11.6 m declarado no planejamento exige `δ≈0.531273749 rad`, acima desse bound. **Não ajustar raio, joint limit, max_steer ou wheelbase para obter PASS.** A inconsistência deve ser resolvida por revisão geométrica/física e posteriormente testada com autorização de movimento. O eixo dianteiro tem ainda uma pequena assimetria longitudinal, enquanto a fórmula do plugin idealiza a geometria Ackermann.

Outro limite é o ponto de referência. `base_footprint` está 3.502 m à frente do centro do eixo traseiro selecionado em x. Para um corpo rígido planar, com `r=(a,b)` do eixo traseiro à base:

\[
v_{B,x}=v_R-\omega b,\qquad v_{B,y}=\omega a.
\]

Com `b≈0`, a velocidade longitudinal coincide, mas a base tem componente lateral durante curvas. MPPI Ackermann integra uma trajetória não holonômica sem `vy` para o ponto controlado. A conversão de unidades sozinha não prova equivalência entre a trajetória desse ponto e a cinemática física do modelo. Escolha de frame de controle, footprint, rota, goal checker e métricas precisa permanecer coerente; não mover silenciosamente a origem histórica.

Na próxima coleta sem movimento, comparar ao mesmo instante `base_footprint→first_wheel_left_link`, `→first_wheel_right_link`, `→fourth_wheel_left_link`, `→fourth_wheel_right_link` e os dois steering links, mantendo os respectivos joint_states. A árvore URDF em posição inicial reproduz os centros acima, à precisão dos literais. TF, contudo, representa links; não comprova sozinho centros de colisão realmente carregados, orientação dos eixos, contato/pneus ou a geometria efetiva interna calculada em `Load()`. A aprovação requer correlacionar também uma inspeção somente de leitura do modelo carregado no Gazebo. A nova variante de TF/sensores deve produzir geometria idêntica no auditor.

## 4. Inversa correta e política necessária antes de uma implementação operacional

Para bicicleta ideal, velocidade longitudinal assinada no eixo traseiro `v≠0` e yaw rate desejado `ω`:

\[
\delta_{ideal}=\arctan(L\omega/v),\qquad
u=\operatorname{sign}(v)\,\delta_{ideal}
=\operatorname{atan2}(L\omega,|v|).
\]

Essa última expressão já compensa a inversão de sinal aplicada pelo plugin. Em ré, `ω>0` exige `δ<0`, mas `u>0`; reaplicar o sinal da ré duas vezes dá o resultado errado. `atan2(Lω,v)` também é inadequado em ré, pois pode selecionar um ângulo fora do ramo principal de esterçamento. O helper matemático `ideal_plugin_input` testa somente essa identidade; não implementa os requisitos operacionais abaixo.

| Situação | Contrato proposto, ainda não autorizado para atuação |
|---|---|
| `v=+0` ou `v=−0`, `ω=0` | Normalizar a parada para zero; não deixar o sinal IEEE de zero selecionar esterçamento. |
| Velocidade nula com `ω≠0` | Rejeitar rotação no lugar e sinalizar pedido não realizável. Não dividir por um epsilon arbitrário. |
| Velocidade próxima de zero | Faixa de parada/histerese depende de resolução e dinâmica validadas; não há valor inventado nesta revisão. A faixa e transição precisam ser configuradas/testadas antes de implementar. |
| Ré e cruzamento por zero | Validar os quatro quadrantes `(±v,±ω)`, reversão do plugin e transições de parada; comparar targets de ambas as rodas e ω esperado. |
| Limites de velocidade/curvatura | Aplicar um contrato conjunto para `(v,ω)` depois do smoother. Alterar somente v mantendo ω pode aumentar a curvatura. Se um clamp de velocidade for autorizado e a curvatura desejada for realizável, recomputar ω=v_aplicado×κ. Curvatura irrealizável exige rejeição ou projeção explicitamente revisada, que muda a trajetória; reduzir v e ω proporcionalmente não corrige uma curvatura impossível. |
| Saturação de direção | Limitar pelos dois joints, ramo válido e geometria verificada, não apenas pelo `max_steer` do plugin. Registrar solicitado/aplicado, motivo e erro de curvatura; não ocultar clipping. |
| NaN, infinito, overflow ou geometria inválida | Invalidar o comando/contrato, inibir movimento e registrar falha; não propagar valores nem reter silenciosamente um alvo antigo. |
| Dados vencidos, clock parado/regressivo, falha de processo | Inibir movimento com watchdog monotônico e mecanismo final independente; não usar somente tempo de simulação para detectar silêncio. Exigir rearmamento deliberado após falha. |
| Múltiplos produtores ou bypass | Saída dedicada com autoridade única até o plugin; prioridade de parada e arbitragem devem cobrir controller, behaviors e qualquer outro produtor. |

No upstream 3.7.0, `OnCmdVel` não guarda timestamp, não valida finitude e não implementa expiração: o último alvo persiste. O timeout 1.0 s do smoother funciona enquanto esse processo e seu relógio estão operacionais; não cobre morte do smoother/adaptador nem substitui proteção no destino. Publicar uma única parada por software não constitui prova de parada física. Antes de conectar um adaptador, é necessário um mecanismo final de expiração/stop independente e uma validação futura de falhas controladas. Esta entrega não modifica o plugin instalado.

Há ainda um caso específico a conferir na versão MPPI 1.2.10: quando sua restrição de raio está ativa, a expressão upstream é `sign(wz) × vx / min_turning_r`. Com `vx<0`, esse resultado tem sinal oposto ao `wz` de entrada; por exemplo `vx=−1`, `wz=+0.2`, `R=11.6` resulta `wz≈−0.0862069`. Isso é uma propriedade do trecho inspecionado, não uma observação de atuação neste host. O contrato de ré deve incluir esse caso e eventual correção deve ser versionada separadamente; não alterar MPPI nem interpretar seu efeito como já validado por estes cálculos.

## 5. Reprodução e critérios de encerramento

No checkout dedicado, o comando completo abaixo executa **apenas a auditoria estática**, com stdlib Python e saída nova:

```bash
audit_root=$(mktemp -d "${TMPDIR:-/tmp}/nav2-actuation-audit.XXXXXXXX")
python3 -B tools/auto_tuning/audit_ackermann_geometry.py \
  src/gps_truck_nav/truck_bringup/models/arocs_truck/model.sdf \
  --requested-turning-radius 11.6 \
  --output "$audit_root/ackermann_geometry_static.json"
```

`STATIC_MODEL_AUDITED` significa que a leitura e o cálculo terminaram, mesmo quando expõem conflitos. `runtime_geometry_validated=false` e `motion_authorized=false` permanecem explícitos. A suíte `tests/test_ackermann_geometry.py` cobre poses relativas/rotacionadas, incompatibilidade de limites, invariância por transformação rígida, falhas de entrada, quatro quadrantes/zero da equação e preservação de evidências. São provas offline de contrato, não testes de atuação.

Considerar a interface resolvida somente após: identidade da biblioteca carregada correlacionada ao contrato; geometria inicial carregada medida e reconciliada com SDF/URDF; limites de ambos os joints/raio/ponto de referência reconciliados; semântica e autoridade dos tópicos demonstradas; especificação de zero/ré/saturações/falhas implementada e testada offline; watchdog final validado; e testes de atuação controlados autorizados em etapa posterior pelo revisor. **A próxima execução continua sendo build/probe sem movimento.**

## Fontes primárias verificadas

- [Gazebo Ackermann 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp): joints opcionais/essenciais 220–294; colisões/centros 329–352; flags TF 401–428; targets/forças 507–543; callback sem timeout 551–556; odometria no frame da base 575–606. Blob `8f536598f837acef619337f4b72b626e7021dfa8`.
- [MPPI optimizer 1.2.10](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/src/optimizer.cpp): integração de yaw e `getControlFromSequenceAsTwist`; blob `86359395ef96081b9858ffc1fd1d61fa762a8d72`.
- [MPPI motion models 1.2.10](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp): `AckermannMotionModel::applyConstraints`; blob `0c5671ea3e56f994c8257d19026c9dfecd8fb34d`.
- [Velocity smoother 1.2.10](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_velocity_smoother/src/velocity_smoother.cpp): validação de Twist, timeout, limites e publicação; blob `d3dd5c2a9979c5863d4106784867d81bd92662cf`.
