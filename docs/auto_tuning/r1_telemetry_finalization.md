# R1 — encerramento normal da telemetria

Correção a partir de `c9390c710769ef8eaadf7b2c1a67cdcdbd54bc66`, na PR #2 DRAFT.
Estado: implementação e regressões offline aprovadas; validação real no Iron do
ipt-sim-01 ainda pendente. Nenhum movimento autorizado por esta entrega.
Gate 0 e Gate 1 continuam `NOT_YET_VERIFIED`; `mission_ready=false`.

Continuação após o resultado real de `dfdc528`: usar o procedimento completo
de [sincronização temporal e nova validação sem movimento](clock_sync_read_only_validation.md).
O relatório abaixo preserva a implementação e os resultados offline da entrega R1.

## Causa e correção

O observer instalava seus handlers e depois chamava `rclpy.init()` com opções
padrão. No contexto padrão do Iron, isso instala handlers ROS para SIGINT e
SIGTERM. O SIGINT usado pela sessão para solicitar o flush podia fechar o
contexto e produzir `ExternalShutdownException`. O host aceitava o motivo de
conclusão da sessão sem conferir o resultado final do observer.

O código oficial consultado foi
[rclpy Iron, inicialização](https://github.com/ros2/rclpy/blob/iron/rclpy/rclpy/__init__.py),
blob `1a200a8be534b4fafb0f146ec7d2b12b546bc1c8`.

Arquivos completos entregues pelo Git, sem inserção manual de trechos:

| Arquivo / função | Mudança |
|---|---|
| `commissioning_observer.py`, `run` | Usa `SignalHandlerOptions.NO`; parada solicitada finaliza JSONL/resumo. Shutdown inesperado, expiração própria, callbacks, dados inválidos e erros de escrita continuam falhando. |
| `commissioning_observer.py`, `Telemetry.flush` | Conta amostras sincronizadas e métricas combinadas pertencentes ao trecho, sem aceitar warmup como evidência dinâmica. |
| `commissioning_supervision.py`, `telemetry_confirmation` e `verify_telemetry_confirmation` | Exigem exit code zero, ausência de timeout, resumo válido, identidade, status e `error=null`; vinculam a confirmação ao SHA-256 do resumo. |
| `commissioning_session.py`, `finalize_processes` e `main` | Mantém SIGKILL imediato da planta e depois SIGINT do observer; prazo de 2 s com heartbeat FINALIZING; conclusão normal falha se a confirmação faltar. |
| `commissioning.py`, `run` | Relê e valida o resumo independentemente da confirmação da sessão. Sem confirmação, não declara coleta normal concluída. |
| `check_r1_evidence.py` | Consolida somente os artefatos salvos do build/probe e das três preparações. Não chama ROS, Docker ou fonte de comandos. |

O contrato normal vale tanto para a preparação sem injeção quanto para a futura
conclusão de um trecho. As provas de SIGSTOP e cancelamentos não exigem uma
finalização normal do observer. Seus critérios de contenção continuam próprios.

A confirmação normal exige `status=TELEMETRY_COLLECTED`, `error=null`, `run_id`
e `trial` correspondentes, contagens positivas de clock/odom/juntas e matching.
Dados inválidos, regressões de clock e descarte por estouro da fila reprovam.
Idades de receipt podem envelhecer depois de matar a planta; isso é distinguido
dos dados inválidos. A supervisão de frescor durante a sessão foi preservada.

Uma futura coleta dinâmica também exige início/âncora válidos dentro da janela
existente de 50 ms, métricas finitas de trajetória e amostras do trecho com
juntas, referência e trajetória combinadas. Isso verifica presença de evidência;
não aprova desempenho, cobertura contínua, localização absoluta ou frenagem.

Geometria, modelo, C++ Ackermann, fonte determinística, limites, autoridade/GIDs,
leases e identidade dos containers foram preservados. O protocolo mantém seu
hash anterior. Os testes anteriores de contenção de c9390c7 continuam evidência
daquela imagem; os novos testes abaixo ficam em outra pasta.

## Regressões offline

554 testes PASS, zero falhas/erros/skips; 125 são de comissionamento. Há 22 testes
adicionais em relação aos 532 anteriores, incluindo:

- SIGINT e SIGTERM solicitados, com flush dos dois JSONL e resumo válido;
- shutdown ROS inesperado, exceção de callback, dados inválidos e escrita falha;
- observer não zero, resumo ausente/corrompido/não finito/de outra identidade;
- confirmação ausente, divergência de hash e expiração sem resumo da sessão;
- timeout de 2 s, SIGKILL antes de flush e continuidade de FINALIZING;
- ausência de métricas do trecho, preservação dos dois fault drills e
  cancelamento antes de RPC Docker bloqueado;
- falha no startup da sessão e consolidação de evidências incompletas.

Os testes usam ROS/processos/Docker simulados em memória, o núcleo C++ real e
um processo descartável da suíte anterior para SIGSTOP. Não iniciam a planta.
Fixtures sintéticas de autorização existentes são apenas testes temporários;
nenhuma autorização operacional foi criada ou preenchida.

Logs: `evidence/r1_unit_tests.log`, `evidence/r1_commissioning_tests.log` e
`evidence/r1_offline_summary.json`, relativos a esta pasta de documentação.

## ipt-sim-01 — checkout, build, probe e três preparações sem movimento

Executar o bloco inteiro no terminal 1. Informar o SHA completo publicado na
entrega R1. Não atualizar o checkout entre as etapas. Cada execução cria uma
pasta nova e usa o cache Docker normal; não há prune, remoção de imagens,
reinício do daemon, sudo ou mudanças em Humble/CARLA/Autoware.

```bash
read -r -p 'SHA completo do novo commit R1 informado na entrega: ' r1_commit
(
  set -euo pipefail
  [[ "$r1_commit" =~ ^[0-9a-f]{40}$ ]]
  test "$r1_commit" != c9390c710769ef8eaadf7b2c1a67cdcdbd54bc66
  task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
  r1_root=$(mktemp -d "$task_root/r1-telemetry.XXXXXXXX")
  printf 'Pasta completa desta tentativa R1: %s\n' "$r1_root"
  printf '%s\n' "$r1_commit" > "$r1_root/checkout_commit.txt"
  git -C "$task_root/gps_truck_nav" fetch origin \
    engineering/2026-10-preflight-gates \
    2>&1 | tee "$r1_root/git_fetch.log"
  git -C "$task_root/gps_truck_nav" merge-base --is-ancestor "$r1_commit" FETCH_HEAD
  GIT_LFS_SKIP_SMUDGE=1 git -C "$task_root/gps_truck_nav" worktree add \
    --detach "$r1_root/gps_truck_nav" "$r1_commit" \
    2>&1 | tee "$r1_root/worktree.log"
  cd "$r1_root/gps_truck_nav"
  git lfs checkout 2>&1 | tee "$r1_root/lfs_checkout.log"
  git lfs fsck 2>&1 | tee "$r1_root/lfs_fsck.log"
  test "$(git rev-parse HEAD)" = "$r1_commit"
  test -z "$(git status --porcelain)"

  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$r1_root/gps_truck_nav" \
    --output-dir "$r1_root/build" \
    2>&1 | tee "$r1_root/build-console.log"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
    --repo "$r1_root/gps_truck_nav" \
    --build-dir "$r1_root/build" \
    --output-dir "$r1_root/probe" \
    --navigation-profile candidate20kmh \
    2>&1 | tee "$r1_root/probe-console.log"

  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$r1_root/gps_truck_nav" \
    --build-dir "$r1_root/build" --probe-dir "$r1_root/probe" \
    --output-dir "$r1_root/prepare-normal" \
    --fault-drill none --duration 45 --port 8765 \
    2>&1 | tee "$r1_root/prepare-normal-console.log"

  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$r1_root/gps_truck_nav" \
    --build-dir "$r1_root/build" --probe-dir "$r1_root/probe" \
    --output-dir "$r1_root/prepare-adapter" \
    --fault-drill adapter --duration 120 --port 8765 \
    2>&1 | tee "$r1_root/prepare-adapter-console.log"

  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$r1_root/gps_truck_nav" \
    --build-dir "$r1_root/build" --probe-dir "$r1_root/probe" \
    --output-dir "$r1_root/prepare-session" \
    --fault-drill session --duration 120 --port 8765 \
    2>&1 | tee "$r1_root/prepare-session-console.log"

  /usr/bin/python3 -I tools/auto_tuning/check_r1_evidence.py \
    --root "$r1_root" --output "$r1_root/r1_evidence_review.json" \
    2>&1 | tee "$r1_root/r1-review-console.log"
  printf 'Coleta R1 concluída; preservar e enviar: %s\n' "$r1_root"
)
```

`set -euo pipefail` interrompe a sequência se qualquer etapa falhar. Nesse caso,
guardar o caminho e os logs; não reutilizar pastas, editar relatórios, preencher
templates ou tentar o movimento. Uma nova tentativa usa outra pasta. A criação
do template falso, feita pelo helper preexistente, não constitui autorização;
ele deve continuar falso e sem preenchimento.

A preparação normal deve terminar pelo prazo, sem Ctrl-C: é ela que prova o
encerramento ordenado real. As duas injeções encerram assim que a prontidão é
observada e podem terminar antes de abrir a página. Não há publisher de atuação
nem fonte de Twist nas três preparações.

Para observar, abrir no navegador do ipt-sim-01 o URL localhost com token
impresso em cada sessão. Não reutilizar o token anterior. Fechar a aba não
encerra o container. Uma sessão anterior ocupando a porta precisa ser encerrada
pelo seu próprio diretório, sem matar outros containers. Não há novos ajustes
visuais ou goals nesta validação.

## Status e cancelamento em outro terminal

Informar o caminho impresso da tentativa e a sessão ativa. Para a primeira
sessão, digitar `prepare-normal`; nas seguintes, `prepare-adapter` ou
`prepare-session`.

```bash
read -r -p 'Pasta completa r1-telemetry desta tentativa: ' r1_root
read -r -p 'Sessão ativa (prepare-normal, prepare-adapter ou prepare-session): ' r1_session
case "$r1_session" in
  prepare-normal|prepare-adapter|prepare-session) ;;
  *) printf 'Nome de sessão inválido\n'; exit 1 ;;
esac
test -n "$r1_root" && test -f "$r1_root/$r1_session/manifest.json" || exit 1
/usr/bin/python3 -I "$r1_root/gps_truck_nav/tools/auto_tuning/commissioning.py" status \
  --session-dir "$r1_root/$r1_session"
```

Se precisar cancelar, usar o bloco seguinte ou Ctrl-C no terminal 1. Uma prova
cancelada não recebe aprovação. A contenção tem prioridade sobre o flush.

```bash
/usr/bin/python3 -I "$r1_root/gps_truck_nav/tools/auto_tuning/commissioning.py" stop \
  --session-dir "$r1_root/$r1_session"
/usr/bin/python3 -I "$r1_root/gps_truck_nav/tools/auto_tuning/commissioning.py" status \
  --session-dir "$r1_root/$r1_session"
```

## Critérios objetivos e consolidação

| Etapa | Exigir |
|---|---|
| Build/probe | IMAGE_VALIDATED e OBSERVATIONS_COLLECTED com mesma fonte/imagem, candidate20kmh. O status do probe não significa saúde global nem Gate aprovado. |
| prepare-normal | PREPARATION_COLLECTED, error=null, prontidão observada, zero publishers, reason=DURATION_LIMIT_REACHED, normal_telemetry_confirmed=true. |
| Sessão normal | telemetry_finalization.status=VERIFIED; observer_returncode=0; timed_out=false; resumo com identidade, status e error corretos; SHA-256 correspondente no host e na sessão. |
| Falha do adaptador/supervisor | Ambas PREPARATION_COLLECTED, fault_drill_passed=true, zero publishers, contenção em menos de 3 s. Resumo do observer pode estar incompleto/ausente por contenção forçada. |
| Todos os containers | OWNED_SIMULATOR_TERMINATED, running=false, pid=0, CID correspondente. Mesma fonte, imagem, modelo e protocolo nas três tentativas. |
| Consolidação | r1_evidence_review.json com status=R1_READ_ONLY_VALIDATED. mission_ready=false e Gates NOT_YET_VERIFIED. |

Depois de completar a sequência, executar este bloco para guardar inventário e
arquivo sem sobrescrever evidências. Ele também pode ser usado após uma falha;
inclui somente os diretórios que chegaram a existir. O checkout fica preservado
fora do arquivo; seu commit é registrado.

```bash
read -r -p 'Pasta completa r1-telemetry a consolidar: ' r1_root
(
  set -euo pipefail
  test -n "$r1_root" && test -f "$r1_root/checkout_commit.txt"
  cd "$r1_root"
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
inventory = Path('r1_inventory_' + stamp + '.json')
with inventory.open('x') as stream:
    json.dump(items, stream, indent=2)
    stream.write('\n')
archive = Path('arocs_r1_review_' + stamp + '.tar.gz')
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

Enviar o arquivo ao revisor. Somente depois dessa coleta será possível confirmar
R1 no runtime. Isso não cria autorização de movimento nem aprova Gate 0/1.
