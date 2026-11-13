# Descompasso entre `/clock`, `/odom` e `/joint_states` — validação sem movimento

Correção incremental sobre `dfdc528832e113768c13630d99d25572eecc0539`, na PR #2
DRAFT. A demonstração visual está concluída. Esta entrega trata somente da
avaliação temporal do adaptador e da prontidão da preparação. Não autoriza
movimento: `mission_ready=false`, Gate 0 e Gate 1 `NOT_YET_VERIFIED`.

## Evidência e limite da conclusão causal

Na tentativa informada pelo operador no ipt-sim-01, o build de `dfdc528` passou,
mas a preparação normal foi contida por
`ADAPTER_FAULT: SENSOR_STAMP_OUTSIDE_CLOCK_WINDOW`, aproximadamente 17,86 s
depois do início. O adaptador não criou publisher de atuação. A contenção
encerrou o container com `Running=false` e `Pid=0`. O observer finalizou com
`TELEMETRY_COLLECTED` e `error=null`; a sessão foi interrompida antes de escrever
seu resumo normal. As duas provas seguintes não foram executadas.

| Observador / dado | Stamp simulado (s) | Monotônico informado (s) |
|---|---:|---:|
| Adaptador: último `/clock` | 15,111822 | Não informado para esse callback |
| Adaptador: último `/odom` e `/joint_states` | 15,215145 | Não informado para esses callbacks |
| Observer independente: `/odom` | 15,215145 | 3388,992950571 |
| Observer independente: `/joint_states` | 15,215145 | 3388,993519428 |
| Adaptador: entrada em FAULT | — | aproximadamente 3388,995064891 |
| Observer independente: `/clock` posterior | 15,218478 | 3388,995958977 |

A idade calculada pelo adaptador era `clock - sensor = -0.103323 s`. Com o
clock posterior observado pelo outro processo, ela seria `+0.003333 s`. Nesse
observer, o clock foi registrado 3,008406 ms depois da odometria e 0,894086 ms
depois do instante aproximado da falha do adaptador.

Isso é compatível com entrega ou processamento assíncrono de tópicos. Não prova
que o adaptador já tivesse recebido o mesmo clock: cada processo tem suas
próprias assinaturas e seu próprio agendamento. Tampouco prova que qualquer
sensor adiantado seja válido. O código anterior comparava os últimos valores
de três callbacks independentes como se fossem um snapshot simultâneo. O
`spin_some` não fornece essa garantia entre tópicos.

Referências oficiais consultadas na investigação:
[helper `rclcpp::spin_some` do Iron](https://github.com/ros2/rclcpp/blob/iron/rclcpp/src/rclcpp/executors.cpp),
[execução de `spin_some_impl`](https://github.com/ros2/rclcpp/blob/iron/rclcpp/src/rclcpp/executor.cpp)
e [semântica de agendamento dos executors](https://github.com/ros2/ros2_documentation/blob/iron/source/Concepts/Intermediate/About-Executors.rst).

O novo diagnóstico registra os stamps e os instantes monotônicos de
**processamento dos callbacks no próprio adaptador**. Esses instantes não são
timestamps de chegada DDS. A evidência do observer continua independente.

Os heartbeats e eventos `CLOCK_SYNC` incluem `clock_s`, `odom_stamp_s`,
`joint_stamp_s`, os três campos `*_callback_monotonic_s`,
`clock_minus_odom_s`, `clock_minus_joint_s`, `clock_sync_pending`,
`clock_sync_started_monotonic_s`, `clock_sync_deadline_monotonic_s`,
`clock_sync_episodes` e `clock_sync_recoveries`. O início e o prazo do último
episódio ficam preservados depois da recuperação; somente
`clock_sync_pending=true` indica uma reconciliação ainda em andamento.

## Política de correção

| Situação | Tratamento |
|---|---|
| Sensor até 100 ms à frente do último clock | Mantém a janela simulada existente; ela não foi ampliada. |
| Sensor além dessa janela, antes de `ARMED`, com os demais contratos saudáveis | Inicia uma reconciliação de no máximo 100 ms monotônicos. A observação fica pendente, sem prontidão nem aceitação de atuação. |
| Origem do prazo | Primeiro instante de processamento entre os sensores que estão adiantados. Novas mensagens não renovam o prazo. Um callback já antigo não ganha 100 ms adicionais. |
| Clock/sensores tornam-se coerentes antes do prazo | Encerra a pendência e exige novamente o dwell de imobilidade original: 1 s de parede e pelo menos 0,5 s de avanço do stamp. |
| Prazo esgota, mesmo se o snapshot mais recente só então parecer coerente | `SENSOR_CLOCK_SYNC_TIMEOUT`, com contenção. |
| Sensor antigo, perda de frescor, regressão temporal, lease ou identidade inválida, movimento inesperado | As respectivas proteções continuam válidas; a espera não as encobre. |
| Qualquer descompasso fora da janela durante `ARMED` | Falha imediata, inclusive no callback de comando. Não há tolerância transitória durante atuação. |

Os 100 ms monotônicos são um orçamento limitado de processamento antes de
atuar, correspondente a cerca de dez intervalos nominais do loop existente de
10 ms. Não são uma nova tolerância do relógio simulado nem uma garantia de
latência DDS. O intervalo é inferior aos watchdogs de comando (250 ms) e de
observação/lease (500 ms), que permanecem independentes. A nova execução no
host deverá comprovar se esse orçamento basta; uma persistência deve falhar,
sem aumentar o limite para obter aprovação.

Não há espera bloqueante de callbacks, extrapolação de clock, alteração de
stamps, troca silenciosa por amostras antigas ou mudança de QoS/taxa dos
sensores. O adaptador mantém sua fase de prontidão durante a reconciliação,
mas `observations_ready=false`; assim, as proteções já associadas a essa fase
continuam rigorosas. A sessão exige observações coerentes para declarar
prontidão e para terminar normalmente uma preparação sem injeção.

Foram preservados geometria, planta, parâmetros Nav2, sensores, visualização,
limites de comando/esterçamento, autoridade por GID, isolamento, watchdogs,
cancelamento independente e finalização R1. Em `prepare`, o adaptador continua
sem publisher de atuação. Os arquivos completos são entregues pelo checkout
Git; não há trechos para inserir manualmente.

## O que foi verificado e o que depende do host

As regressões offline exercitam o núcleo C++ real com relógios e mensagens
sintéticos, incluindo o descompasso exato de −103,323 ms, recuperação após
atualização legítima de clock, persistência, expiração, proteção de autoridade
e falhas durante atuação. Isso não inicia ROS, Gazebo ou movimento. Os logs e
o relatório offline desta correção são entregues junto ao commit.
O replay usa os tempos informados como uma ordem plausível de callbacks
sintéticos; não reconstrói a ordem real, ainda desconhecida, no adaptador.

Resultado: **594 testes PASS**, zero falhas, erros ou skips. Inclui 40 novas
regressões (34 casos do Guard C++ compilado com `-Werror` e 6 de prontidão da
sessão), além dos testes anteriores de contenção e finalização R1. O smoke
offline incluído na imagem também passou no núcleo C++ e agora reproduz o
snapshot informado, a recuperação e o timeout persistente. Os quatro blocos
Bash abaixo passaram em `bash -n`; o Python de consolidação passou em análise
sintática. Evidências completas em
[`clock_sync_unit_tests.log`](evidence/clock_sync_unit_tests.log) e
[`clock_sync_offline_summary.json`](evidence/clock_sync_offline_summary.json).

Ainda dependem do ipt-sim-01: compilação do wrapper com ROS 2 Iron na imagem,
ordenação real dos callbacks, recuperação temporal quando necessária,
encerramento normal R1 e repetição das duas provas de contenção na nova imagem.
Um contador de reconciliações igual a zero é aceitável se não houve descompasso;
não se deve injetar atraso ou movimento para forçar esse contador.

## Terminal 1 — checkout novo, build, probe e três preparações

Copiar o bloco inteiro e informar o SHA completo publicado nesta entrega.
O bloco cria uma pasta nova. Não modifica o checkout anterior nem apaga
evidências ou imagens. Usa o cache Docker padrão, sem `--no-cache`, prune,
sudo, reinício do daemon ou alterações em Humble/CARLA/Autoware.

```bash
read -r -p 'SHA completo do novo commit de sincronização temporal: ' clock_commit
(
  set -euo pipefail
  [[ "$clock_commit" =~ ^[0-9a-f]{40}$ ]]
  test "$clock_commit" != dfdc528832e113768c13630d99d25572eecc0539
  task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
  clock_root=$(mktemp -d "$task_root/clock-sync.XXXXXXXX")
  printf 'Pasta completa desta tentativa: %s\n' "$clock_root"
  printf '%s\n' "$clock_commit" > "$clock_root/checkout_commit.txt"
  git -C "$task_root/gps_truck_nav" fetch origin \
    engineering/2026-10-preflight-gates \
    2>&1 | tee "$clock_root/git_fetch.log"
  git -C "$task_root/gps_truck_nav" merge-base --is-ancestor "$clock_commit" FETCH_HEAD
  git -C "$task_root/gps_truck_nav" merge-base --is-ancestor \
    dfdc528832e113768c13630d99d25572eecc0539 "$clock_commit"
  GIT_LFS_SKIP_SMUDGE=1 git -C "$task_root/gps_truck_nav" worktree add \
    --detach "$clock_root/gps_truck_nav" "$clock_commit" \
    2>&1 | tee "$clock_root/worktree.log"
  cd "$clock_root/gps_truck_nav"
  git lfs checkout 2>&1 | tee "$clock_root/lfs_checkout.log"
  git lfs fsck 2>&1 | tee "$clock_root/lfs_fsck.log"
  test "$(git rev-parse HEAD)" = "$clock_commit"
  test -z "$(git status --porcelain)"

  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$clock_root/gps_truck_nav" \
    --output-dir "$clock_root/build" \
    2>&1 | tee "$clock_root/build-console.log"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
    --repo "$clock_root/gps_truck_nav" \
    --build-dir "$clock_root/build" \
    --output-dir "$clock_root/probe" \
    --navigation-profile candidate20kmh \
    2>&1 | tee "$clock_root/probe-console.log"

  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$clock_root/gps_truck_nav" \
    --build-dir "$clock_root/build" --probe-dir "$clock_root/probe" \
    --output-dir "$clock_root/prepare-normal" \
    --fault-drill none --duration 45 --port 8765 \
    2>&1 | tee "$clock_root/prepare-normal-console.log"

  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$clock_root/gps_truck_nav" \
    --build-dir "$clock_root/build" --probe-dir "$clock_root/probe" \
    --output-dir "$clock_root/prepare-adapter" \
    --fault-drill adapter --duration 120 --port 8765 \
    2>&1 | tee "$clock_root/prepare-adapter-console.log"

  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$clock_root/gps_truck_nav" \
    --build-dir "$clock_root/build" --probe-dir "$clock_root/probe" \
    --output-dir "$clock_root/prepare-session" \
    --fault-drill session --duration 120 --port 8765 \
    2>&1 | tee "$clock_root/prepare-session-console.log"

  /usr/bin/python3 -I tools/auto_tuning/check_r1_evidence.py \
    --root "$clock_root" --output "$clock_root/r1_evidence_review.json" \
    2>&1 | tee "$clock_root/r1-review-console.log"
  printf 'Coleta concluída; preservar e enviar: %s\n' "$clock_root"
)
```

`set -euo pipefail` interrompe a sequência na primeira falha. Não continuar
manualmente as etapas seguintes para obter um relatório verde. Preservar o
caminho impresso e consolidar a tentativa, inclusive se incompleta. Para
repetir, executar novamente o bloco com outra pasta criada por `mktemp`.

A preparação normal deve terminar pelo prazo, sem Ctrl-C: é a prova de
finalização ordenada. As injeções de falha do adaptador e do supervisor usam
SIGSTOP e terminam assim que demonstram a contenção; podem terminar antes de
abrir o navegador. Não dependem de uma finalização normal impossível depois
da injeção. Nenhuma das três preparações inicia fonte de Twist ou publisher
de atuação. Não criar nem preencher autorização operacional. O template falso
produzido pelo helper preexistente deve permanecer falso e intacto.

Para observar, abrir no navegador do ipt-sim-01 o URL localhost com token
impresso pela sessão ativa. Não reutilizar o token anterior. Fechar a aba não
encerra o container. Se houver uma sessão anterior na porta 8765, encerrá-la
pelo diretório dessa sessão; não matar containers de outros ambientes.

## Terminal 2 — status e cancelamento independente

Informar a pasta completa `clock-sync.XXXXXXXX` impressa no terminal 1 e o nome
da preparação ativa.

```bash
read -r -p 'Pasta completa clock-sync desta tentativa: ' clock_root
read -r -p 'Sessão ativa (prepare-normal, prepare-adapter ou prepare-session): ' clock_session
case "$clock_session" in
  prepare-normal|prepare-adapter|prepare-session) ;;
  *) printf 'Nome de sessão inválido\n'; exit 1 ;;
esac
test -n "$clock_root" && test -f "$clock_root/$clock_session/manifest.json" || exit 1
/usr/bin/python3 -I "$clock_root/gps_truck_nav/tools/auto_tuning/commissioning.py" status \
  --session-dir "$clock_root/$clock_session"
```

Se precisar cancelar, executar o bloco seguinte nesse terminal, ou Ctrl-C no
terminal 1. A contenção tem prioridade sobre o salvamento da telemetria. Uma
preparação cancelada não aprova esta validação.

```bash
/usr/bin/python3 -I "$clock_root/gps_truck_nav/tools/auto_tuning/commissioning.py" stop \
  --session-dir "$clock_root/$clock_session"
/usr/bin/python3 -I "$clock_root/gps_truck_nav/tools/auto_tuning/commissioning.py" status \
  --session-dir "$clock_root/$clock_session"
```

## Critérios de aprovação e revisão

| Evidência | Exigir |
|---|---|
| Build e probe | `IMAGE_VALIDATED`, `OBSERVATIONS_COLLECTED`, perfil `candidate20kmh`, mesmo commit e imagem. Coleta de observações não significa aprovação integral do Nav2. |
| Normal, sem injeção | `PREPARATION_COLLECTED`, `error=null`, `reason=DURATION_LIMIT_REACHED`, prontidão observada e observações coerentes no encerramento. Uma reconciliação pendente não pode concluir normalmente. |
| Reconciliação temporal, se ocorrer | Stamps e instantes de processamento preservados; episódios limitados por prazo fixo, recuperados antes de 100 ms monotônicos; nenhuma autorização de atuação durante pendência. |
| Telemetria R1 | `normal_telemetry_confirmed=true`, `telemetry_finalization.status=VERIFIED`, `observer_returncode=0`, `timed_out=false`; resumo com identidade/status/error corretos e SHA-256 igual na sessão e no host. |
| Ambas as injeções | `PREPARATION_COLLECTED`, `fault_drill_passed=true`, contenção em menos de 3 s. Resumo normal do observer não é requisito dessas injeções. |
| Todas as preparações | Zero publishers de atuação; `OWNED_SIMULATOR_TERMINATED`, `running=false`, `pid=0`, CID correspondente; mesma fonte, imagem, modelo e protocolo. |
| Consolidação automática | `r1_evidence_review.json` com `status=R1_READ_ONLY_VALIDATED`, `mission_ready=false`, Gates `NOT_YET_VERIFIED`. |

Revisar também `prepare-normal/adapter/adapter_events.jsonl` e
`adapter_heartbeat.json` dessa mesma pasta de adaptador. A passagem pelo checker
R1 vincula o encerramento normal e a contenção às identidades salvas; não é
validação dinâmica, autorização de uso único ou aprovação de Gate 0/1.

## Consolidar a tentativa sem sobrescrever evidências

O bloco abaixo funciona também depois de uma falha e inclui os diretórios que
chegaram a existir. O checkout e todas as evidências originais permanecem no
host. O arquivo leva um timestamp novo e é aberto em modo exclusivo.

```bash
read -r -p 'Pasta completa clock-sync a consolidar: ' clock_root
(
  set -euo pipefail
  test -n "$clock_root" && test -f "$clock_root/checkout_commit.txt"
  cd "$clock_root"
  /usr/bin/python3 -I - <<'PY'
import hashlib
import json
from pathlib import Path
import tarfile
from datetime import datetime, timezone

directories = ('build', 'probe', 'prepare-normal', 'prepare-adapter', 'prepare-session')
items = []
for directory in directories:
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and not path.is_symlink():
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(1048576), b''):
                    digest.update(block)
            items.append({'path': str(path), 'bytes': path.stat().st_size, 'sha256': digest.hexdigest()})
stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
inventory = Path('clock_sync_inventory_' + stamp + '.json')
with inventory.open('x') as stream:
    json.dump(items, stream, indent=2)
    stream.write('\n')
archive = Path('arocs_clock_sync_review_' + stamp + '.tar.gz')
with tarfile.open(archive, 'x:gz') as bundle:
    for directory in directories:
        if Path(directory).exists():
            bundle.add(directory)
    for path in sorted(Path('.').glob('*.log')):
        bundle.add(path)
    bundle.add('checkout_commit.txt')
    bundle.add(inventory)
    if Path('r1_evidence_review.json').is_file():
        bundle.add('r1_evidence_review.json')
digest = hashlib.sha256()
with archive.open('rb') as stream:
    for block in iter(lambda: stream.read(1048576), b''):
        digest.update(block)
print('Arquivo:', archive.resolve())
print('SHA-256:', digest.hexdigest())
PY
)
```

Enviar o arquivo e seu SHA-256 ao revisor independente. A meta desta coleta é
`R1_READ_ONLY_VALIDATED` na imagem corrigida. Movimento continua dependendo de
uma autorização explícita posterior e separada.
