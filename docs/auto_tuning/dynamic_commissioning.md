# Comissionamento dinâmico prospectivo do Arocs

Para a correção R1 e a próxima validação exclusivamente sem movimento, seguir
[R1 — finalização normal da telemetria](r1_telemetry_finalization.md).
Ela exige confirmação do observer na conclusão normal, preservando os fault
drills e a contenção existentes. O runtime da revisão R1 ainda precisa ser
verificado no ipt-sim-01 antes de qualquer autorização posterior de movimento.

Estado desta entrega: código e testes offline; **nenhum movimento executado**.
O operador confirmou a demonstração visual de `12c6ff8` no ipt-sim-01: Arocs
completo no Gazebo/RViz, CTVI e footprint. Esse marco está concluído. Esta revisão
não altera câmera, malhas, sensores, RViz, geometria ou parâmetros físicos.
Gate 0 e Gate 1 continuam **NOT_YET_VERIFIED**; `mission_ready=false`.

## 1. Implementação e limites da evidência

| Arquivo completo | Função / ponto de entrada |
|---|---|
| `tools/auto_tuning/commissioning.py` | CLI do host; `run`, `stop_owned` e `main`: preparação, autorização futura, supervisão e encerramento exclusivo do container |
| `tools/auto_tuning/commissioning_session.py` | `main`: Gazebo/RViz, launch isolado, adaptador, telemetria e supervisor sem dependência de callbacks ROS |
| `tools/auto_tuning/commissioning_adapter/guard.hpp` | `convert` e `Guard`: conversão, envelopes, autoridade, frescor, estados e falhas terminais |
| `tools/auto_tuning/commissioning_adapter/adapter.cpp` | `main`: ROS Iron, GID por mensagem, leitura do grafo, publicação final e evidências |
| `tools/auto_tuning/commissioning_source.py` | `protocol`, `command_at`, `execute`: um único trecho determinístico, somente mediante autorização e início explícito |
| `tools/auto_tuning/commissioning_observer.py` | `Telemetry` e `run`: odometria/juntas/referência sincronizadas e métricas |
| `tools/auto_tuning/commissioning_supervision.py` | `HeartbeatWatch`, `verify_authorization`: leases, identidade e vínculo da autorização |
| `src/gps_truck_nav/truck_bringup/launch/navigation_launch.py` | Argumento opt-in `actuation_routing:=isolated`, nos dois modos de composição |
| `src/gps_truck_nav/truck_bringup/launch/gps_waypoint_follower.launch.py` | Declara e encaminha o mesmo argumento; default histórico `legacy` preservado |
| `docker/auto_tuning/Dockerfile`, `tools/auto_tuning/validate_image.sh` | Compilação/linkagem do adaptador e self-test anterior a `rclcpp::init`, sem DDS/publishers |

Os arquivos são entregues inteiros pelo Git; não inserir trechos manualmente.
O adaptador Python shadow anterior continua receive-only e não foi convertido
silenciosamente em um caminho de atuação. O adaptador operacional é C++ porque
Iron rclcpp expõe o GID real de cada mensagem; rclpy Iron não o oferece no
contrato já investigado. Isso permite comparar a origem da mensagem com o
endpoint único observado e fixado para a sessão.

O núcleo C++ é compilado e exercitado offline com g++, inclusive limites,
histerese, autoridade/GID, relógios, leases, perda de comandos, ré, zeros,
falhas e parada observada. Há regressões Python do host, observador, fonte,
roteamento e supervisor; uma prova usa um processo descartável real com SIGSTOP,
sem ROS/Docker. A execução da fonte também foi testada com objetos ROS simulados
em memória. Esses testes não comprovam DDS, linkagem Iron ou resposta física.
Logs e resumo reproduzíveis: `evidence/commissioning_tests.log`,
`evidence/commissioning_checks.log`, `evidence/commissioning_summary.json`.

**Ainda exige ipt-sim-01:** build do wrapper contra Iron; self-test na imagem;
observação de endpoints e sincronização reais; dois testes de perda de processo
com zero publishers de atuação; revisão da área livre. Só depois de autorização
explícita: sinal das rodas, velocidade realizada, curvatura, escorregamento,
resposta dos PIDs e parada com o caminhão em movimento.

## 2. Caminho de comando e proteção

O primeiro ensaio usa exclusivamente:

`phase_a_commissioning_source → /commissioning/cmd_vel → phase_a_ackermann_guard → /actuation/steering_command → ackermann_drive`.

O comando de entrada é Twist `(v, ω)` no referencial `rear_axle_nav`.
O último Twist transporta a interface específica do Classic: `linear.x` é o
alvo longitudinal da roda traseira direita, e `angular.z` é esterçamento,
**não yaw rate**. Para o modelo prospectivo preservado:

- `L=6.816500073351341 m`, `W=2.598120190224205 m`, `y_RR=-1.299 m`;
- `v_plugin = v_rear − ω*y_RR`;
- `δ=atan(L*ω/v_rear)`; em avanço, `u_plugin=δ`;
- os dois alvos de juntas são calculados pela mesma relação Ackermann do plugin;
- ré é explicitamente rejeitada neste comissionamento. O contrato matemático de
  ré anterior permanece rastreável, mas não constitui autorização de ensaio.

Não há clipping silencioso. Excesso de velocidade/yaw, curvatura impossível,
envelope de juntas ou componentes Twist não suportadas geram falha latched e
encerramento; o comando solicitado/rejeitado e o motivo ficam no JSONL. Não
reduzir limites físicos ou modificar os PIDs para fazer os testes passarem.

O novo roteamento Nav2 prepara `controller → cmd_vel_nav → smoother →
/actuation/nav2_twist`; behaviors ficam em `/actuation/behavior_quarantine`.
**O adaptador deste primeiro ensaio não aceita a saída Nav2.** Qualquer comando
não nulo observado nesses caminhos, `/cmd_vel` ou `/cmd_vel_nav` interrompe o
comissionamento. A integração de unidades e o ponto após o smoother estão
preparados; habilitar Nav2 como fonte será uma etapa explícita posterior à
validação da planta. Não há goal, teleop, missão, Virtual Lanes ou Optuna aqui.

| Proteção | Contrato implementado |
|---|---|
| Preparação | Nenhum publisher de atuação; nenhum test source, nem publicação de zero |
| Autoridade | Uma fonte nomeada, GID por mensagem igual ao endpoint fixado; um publisher final, um subscriber final `/ackermann_drive`; alterações/conflitos abortam |
| Envelopes de comando | `v≤0.5 m/s`, `|ω|≤0.02 rad/s`, `|alvo de cada junta|≤0.20 rad`; sobrepostos aos limites físicos existentes |
| Envelopes observados | Velocidade planar `≤0.7 m/s`, `|ω|≤0.05 rad/s`, `|junta|≤0.25 rad`, deslocamento e percurso acumulado `≤10 m` |
| Zero e início | Faixa de parada `0.01 m/s`, retomada `0.02 m/s`; sem yaw em velocidade nula; antes de armar, parada planar e yaw `≤0.005 rad/s` por pelo menos 1 s de parede e 0.5 s de stamps avançando |
| Comandos | Expiração monotônica em 0.25 s; sem retransmissão automática do último alvo; novo comando tardio não rearma |
| Dados | Receipt e stamps de odom/juntas avançando; prazo 0.5 s; comparação com `/clock` e tolerância explícita de reordenação de 0.1 s; regressão/jump de clock >0.25 s aborta |
| Supervisor do adaptador | Processo separado; lease de parede, heartbeat com run ID, sequência e timestamp; falha de processo, fonte ou telemetria encerra a planta |
| Supervisor do host | Independente de ROS e do servidor HTTP; vigia o supervisor interno. Se o host deixar de renovar a lease, o supervisor interno encerra a sessão |
| Contenção final | SIGKILL do grupo do bringup e/ou `docker kill` somente no CID identificado; prova final exige `Running=false`, `Pid=0`; sem remover container/imagem |
| Autorização | Imagem, commit, modelo, protocolo, run ID, área livre, operador, referência e validade de no máximo uma hora; uso único; nenhum template vem autorizado |
| Início e cancelamento | Execução futura exige CLI `execute`, autorização e confirmação textual `START` após imagem ao vivo e telemetria prontas. Ctrl-C ou comando `stop` solicita contenção; `stop` grava cancelamento para o guard antes de qualquer RPC Docker |

Os envelopes são limites conservadores do **procedimento inicial**, não novos
ratings do veículo. Devem ser avaliados com as medições; não aumentá-los apenas
para remover falhas. A autoridade ROS vale no conjunto de processos confiáveis
deste container sem rede externa; não é uma garantia criptográfica contra um
processo malicioso. Twist não contém timestamp de origem: receipt monotônico,
GID e fonte própria limitada são evidências distintas, sem equivalência inventada.

O plugin upstream não tem timeout e aplica o mesmo torque às rodas traseiras,
medindo a velocidade da direita. Portanto zerar por software não prova frenagem.
**Matar a simulação é contenção, não uma medida de distância de parada.** Falha
do Docker/host inteiro não tem garantia temporal de software. A rotina registra
`INDEPENDENT_STOP_NOT_CONFIRMED` se não comprovar o encerramento.

Fontes primárias relidas nesta revisão:
[Gazebo Ackermann 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp),
blob `8f536598f837acef619337f4b72b626e7021dfa8`, controle/callback/odometria;
[rclcpp MessageInfo Iron](https://github.com/ros2/rclcpp/blob/iron/rclcpp/include/rclcpp/message_info.hpp),
blob `452712afdb869f86ab5ebf0fcc45035e644ae6c3`.

## 3. Preparação completa no ipt-sim-01 — sem movimento

Encerrar a sessão visual antiga usando seu próprio helper e diretório, se ainda
estiver aberta. Não usar `docker kill $(docker ps ...)`, prune, rm ou sudo.
Os comandos abaixo reaproveitam o repositório e o cache existentes. A base APT
não foi alterada; a camada do pacote é reconstruída para instalar o novo launch,
e o adaptador é compilado em camada posterior. Cache depende do digest da base.

No terminal 1, executar o bloco inteiro. Ele fixa o commit obtido pelo fetch;
a resposta da entrega informa também o SHA exato para fixação explícita.

```bash
task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
commission_root=$(mktemp -d "$task_root/commissioning.XXXXXXXX")
printf 'Evidências novas: %s\n' "$commission_root"
(
  set -euo pipefail
  git -C "$task_root/gps_truck_nav" fetch origin \
    engineering/2026-10-preflight-gates \
    2>&1 | tee "$commission_root/git_fetch.log"
  commissioning_commit=$(git -C "$task_root/gps_truck_nav" rev-parse FETCH_HEAD)
  printf '%s\n' "$commissioning_commit" > "$commission_root/checkout_commit.txt"
  GIT_LFS_SKIP_SMUDGE=1 git -C "$task_root/gps_truck_nav" \
    worktree add --detach "$commission_root/gps_truck_nav" "$commissioning_commit"
  cd "$commission_root/gps_truck_nav"
  git lfs checkout 2>&1 | tee "$commission_root/lfs_checkout.log"
  git lfs fsck 2>&1 | tee "$commission_root/lfs_fsck.log"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$commission_root/gps_truck_nav" \
    --output-dir "$commission_root/build"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
    --repo "$commission_root/gps_truck_nav" \
    --build-dir "$commission_root/build" \
    --output-dir "$commission_root/probe" \
    --navigation-profile candidate20kmh
)
```

Não avançar se o bloco falhar. A validação da imagem inclui o novo executável,
linkagem e `--offline-self-test`, sem inicializar ROS. O probe original continua
sem atuação e não aprova navegação. Não atualizar esse checkout entre etapas.

Com `IMAGE_VALIDATED` e probe coletado, preparar uma sessão gráfica e provar a
perda do adaptador. Esta etapa **não cria publisher de atuação**. A injeção usa
SIGSTOP somente no PID do adaptador criado dentro do container desta tentativa.

```bash
(
  set -euo pipefail
  cd "$commission_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$commission_root/gps_truck_nav" \
    --build-dir "$commission_root/build" \
    --probe-dir "$commission_root/probe" \
    --output-dir "$commission_root/prepare-adapter" \
    --fault-drill adapter --duration 120 --port 8765 \
    2>&1 | tee "$commission_root/prepare-adapter-console.log"
)
```

O helper imprime um URL `http://127.0.0.1:8765/<token>/`; abrir no navegador do
ipt-sim-01. O teste injeta a falha assim que GUI, sensores e standstill estão
prontos, e pode terminar antes de abrir a aba. Isso é esperado no fault drill;
esta não é uma nova rodada de ajustes visuais. O teste posterior prova a perda
do supervisor interno, deixando o host demonstrar a contenção independente:

```bash
(
  set -euo pipefail
  cd "$commission_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$commission_root/gps_truck_nav" \
    --build-dir "$commission_root/build" \
    --probe-dir "$commission_root/probe" \
    --output-dir "$commission_root/prepare-session" \
    --fault-drill session --duration 120 --port 8765 \
    2>&1 | tee "$commission_root/prepare-session-console.log"
)
```

Se for necessário apenas observar a preparação sem injeção, usar outra pasta
nova e `--fault-drill none`; isso não substitui as duas provas acima. Cada
sessão termina pelo prazo de parede. Fechar a aba não encerra o simulador.
Ctrl-C no terminal do helper encerra somente seu container.

Em outro terminal, sem depender da variável anterior, informar o diretório
impresso da tentativa e executar status ou encerramento. O bloco completo abaixo
consulta e encerra a sessão de teste do adaptador; para a segunda, usar
`prepare-session` em `session_dir`.

```bash
read -r -p 'Pasta completa commissioning desta tentativa: ' commission_root
test -n "$commission_root" || exit 1
session_dir="$commission_root/prepare-adapter"
/usr/bin/python3 -I "$commission_root/gps_truck_nav/tools/auto_tuning/commissioning.py" status \
  --session-dir "$session_dir"
/usr/bin/python3 -I "$commission_root/gps_truck_nav/tools/auto_tuning/commissioning.py" stop \
  --session-dir "$session_dir"
```

O encerramento manual pode interromper uma prova antes de sua conclusão; nesse
caso ela não recebe PASS. Não reutilizar a pasta nem apagar as evidências.

## 4. Critérios de preparação e arquivos para revisão

Para cada fault drill exigir no `commissioning_report.json`:

- `status=PREPARATION_COLLECTED`, `error=null`;
- `read_only_ready_observed=true`, `actuator_publishers_created=0`;
- `fault_drill` correto, `fault_drill_triggered=true`, `fault_drill_passed=true`;
- contenção em menos de 3 s de parede após a injeção, `Running=false`, `Pid=0`;
- mesma fonte, imagem e modelo nas duas tentativas; nenhuma aprovação dinâmica.

Os prazos são critérios do mecanismo de supervisão, não duração de frenagem.
O JSON pode demonstrar que a coleta foi feita e ainda expor uma falha. Não
interpretar `OBSERVATIONS_COLLECTED`, `PREPARATION_COLLECTED` ou `TRIAL_COLLECTED`
como aprovação de Gate, nem transformar NOT_EXERCISED em PASS.

Guardar/enviar: `build`, `probe`, os dois diretórios `prepare-*`, logs de console,
commit e comandos. Manifestos vinculam hashes; configurações consumidas ficam em
`adapter_parameters.json` e no evento CONFIGURATION de `adapter_events.jsonl`.
`authorization.template.json` vem com autorização falsa e expiração zero.
**Não preenchê-lo nem chamar `execute` nesta entrega.**

```bash
(
  set -euo pipefail
  cd "$commission_root"
  /usr/bin/python3 - <<'PY'
import hashlib
import json
from pathlib import Path
items = []
for directory in ('build', 'probe', 'prepare-adapter', 'prepare-session'):
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and not path.is_symlink():
            h = hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(1048576), b''):
                    h.update(block)
            items.append({'path': str(path), 'bytes': path.stat().st_size, 'sha256': h.hexdigest()})
with Path('commissioning_evidence_manifest.json').open('x') as stream:
    json.dump(items, stream, indent=2)
    stream.write('\n')
PY
  archive_path="$commission_root/commissioning-review-$(date -u +%Y%m%dT%H%M%SZ).tar.gz"
  test ! -e "$archive_path"
  tar -czf "$archive_path" \
    checkout_commit.txt commissioning_evidence_manifest.json \
    build probe prepare-adapter prepare-session \
    prepare-adapter-console.log prepare-session-console.log
  sha256sum "$archive_path"
)
```

## 5. Protocolo do primeiro movimento — somente após autorização posterior

1. Revisor avalia os dois fault drills e a identidade da planta/imagem. Operador
   confirma corredor livre cobrindo o footprint e o limite de deslocamento de
   10 m, com margem lateral; mantém o terminal de cancelamento disponível.
2. Registrar autorização explícita para **um trecho reto**, imagem/modelo/run ID
   exatos e validade curta. O helper recusa template falso, expiração, fonte
   divergente, provas incompletas ou reutilização. Uma tentativa consome o run ID,
   inclusive se abortada; nova tentativa exige nova preparação/autorização.
3. Na execução futura, abrir o URL e confirmar imagem atual, área livre e
   telemetria. Só então digitar `START`. Antes disso não há fonte de comandos.
   O adaptador exige fonte única, zero fresco e standstill para armar.
4. Fonte solicita aceleração de **0.1 m/s² durante 3 s**, **0.3 m/s durante 8 s**
   e desaceleração de 3 s, seguida de pelo menos 3 s de zeros. Distância nominal
   solicitada cerca de **3.3 m**. O resultado realizado é medido; 20 km/h não é
   usado. A histerese e a faixa de zero são registradas, não escondidas.
5. Revisar sinais, velocidade longitudinal/lateral traseira, yaw, rodas,
   deslocamento, trajetória e parada observada. Qualquer envelope excedido,
   autoridade divergente, perda de clock/dados/telemetria/GUI ou watchdog aborta.
   A conclusão exige fonte COMPLETE e standstill fresco por 1 s; então a
   simulação é encerrada. Sem standstill, o prazo encerra a simulação e reprova
   o trecho. `physical_stop_verified` continua falso até revisão das medições.
6. Só após aceitar o trecho reto, autorizar **outro ensaio separado** para uma
   curva pequena: `κ=+0.01 m⁻¹` ou `−0.01 m⁻¹` (R=100 m), com rampas de curvatura
   de 1 s apenas no patamar de 0.3 m/s. Não combinar esquerda/direita nem ré
   automaticamente. Alvo no patamar: `|ω|=0.003 rad/s`, aproximadamente
   `|δ|=0.06806 rad`; valores das duas juntas e alvo da roda direita são distintos.
7. Antes de Nav2/goal: revisar repetibilidade e resíduos frente às relações
   cinemáticas, resposta dos PIDs, distância/tempo de parada e falhas controladas
   durante movimento. Autorizar separadamente a fonte Nav2 após o smoother.
   Nenhuma falha de planta é resolvida reajustando YAML para produzir PASS.

O observer registra velocidade longitudinal e lateral no eixo traseiro, yaw
solicitado/observado, ângulos e velocidades das rodas, curvatura realizada
(indefinida perto de zero), erro lateral/posição/heading relativo à trajetória
solicitada e timestamps. O matching admite até 50 ms de diferença entre stamps;
amostras ausentes/desalinhadas permanecem explícitas. Conversões, alvos e
rejeições/saturações estão no JSONL do adaptador. A fonte publica uma referência
stamped observacional; isso não dá timestamp retroativo ao Twist recebido.

A odometria do plugin não é verdade terrestre independente; a projeção é planar
e não comprova localização absoluta, datum, GPS/IMU ou percepção de obstáculos.
Esses limites permanecem na revisão anterior. Esta fase avalia a interface e a
resposta básica da mesma planta prospectiva, preservando sua configuração para
os futuros ensaios referência versus parâmetros otimizados.
