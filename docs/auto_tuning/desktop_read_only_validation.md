# Gazebo Classic e RViz interativos — preparação no desktop

Esta entrega acrescenta `prepare --gui-mode desktop` à PR #2 DRAFT. O modo
padrão continua `xvfb`; `execute` não aceita o novo modo. A planta prospectiva
`candidate20kmh`, a geometria, os parâmetros Nav2, o isolamento de comandos e
os limites de comissionamento permanecem os mesmos.

O operador informou a aprovação de `R1_READ_ONLY_VALIDATED` no ipt-sim-01 para
`168a1fe69cbadb36c75458f799bf0dc3a6b5aacf`: finalização normal confirmada,
contenção da falha do adaptador em 0,836 s e do supervisor em 0,886 s, zero
publishers de atuação, containers encerrados e nenhum movimento. Esses
resultados ficam preservados para aquela imagem. A demonstração visual do
Arocs também está concluída. O novo teste verifica interação e supervisão do
desktop, sem reabrir ajustes cosméticos.

## Implementação e alcance

O host fornece somente o socket Unix do display selecionado e uma credencial
X11 nova, exclusiva da sessão, em mounts somente leitura. A credencial original
do usuário não é montada no container. ROS 2 Iron, Gazebo, Nav2 e RViz
continuam dentro do Docker com rede `none`, `ROS_LOCALHOST_ONLY=1` e domínio
71. Não há `xhost +`, modo privilegiado, rede do host, socket Docker dentro do
container, compartilhamento do socket Wayland ou alteração global de permissões.

Um processo guardião do host cria e revoga a credencial pelo protocolo
XSECURITY. Ela fica em diretório privado de `XDG_RUNTIME_DIR`, fora das
evidências. O guardião encerra o acesso no fechamento do canal com o supervisor
ou no limite absoluto de duração da sessão + 30 s. A expiração XSECURITY de
60 s é **por ociosidade**, não um limite absoluto; por isso a revogação explícita
continua obrigatória. A contenção da planta acontece antes de esperar pela
revogação ou pelo salvamento de evidências.

A credencial é `trusted` para permitir os clientes GLX. Isso não isola o
container dos outros clientes X11 do display enquanto a credencial estiver
válida. A redução de exposição é por credencial própria, mounts mínimos e
vida limitada, sem alegar isolamento por janela. Referências oficiais:
[extensão SECURITY](https://www.x.org/releases/X11R7.7/doc/xextproto/security.html)
e [XWayland](https://wayland.freedesktop.org/docs/book/Xwayland.html).

Em Wayland, o modo usa o `DISPLAY` do XWayland com Qt/xcb. O diagnóstico exige
sessão local autorizada, socket Unix, credencial legível, controle de acesso
ativo e extensões SECURITY/GLX. Ausência ou incompatibilidade encerra a tentativa
antes de criar a simulação. Não há fallback silencioso para capturas web.

O monitor procura somente janelas vinculadas ao hostname exclusivo do
container, PID e grupo de processos de Gazebo/RViz. As consultas X11 rodam em
processo com timeout; não bloqueiam o laço de supervisão. Minimizar ou alternar
janelas é permitido. O monitor não reposiciona janelas do host e não depende de
`Xvfb_screen0`. Perder uma janela já confirmada ou seu processo provoca contenção.

O RViz recebe uma cópia de configuração própria da sessão com apenas
`rviz_default_plugins/MoveCamera`; os displays e os arquivos versionados de
visualização permanecem intactos. Não são adicionadas ferramentas de goal,
pose inicial ou interação com controles de atuação.

**O gzclient nativo ainda contém controles de edição do mundo.** Esta entrega
não transforma esses controles em uma interface tecnicamente incapaz de alterar
a planta. O protocolo permitido é usar somente as câmeras: não pausar/resetar a
simulação, mover modelos, aplicar forças, alterar física ou enviar goals.
Movimento inesperado e regressão/perda do relógio continuam acionando as
proteções. A interação de câmera não cria autoridade ROS de movimento.

## 1. Terminal local — checkout novo e diagnóstico do display

Abrir um terminal na própria sessão gráfica Ubuntu do ipt-sim-01, como Marcus,
sem sudo e sem SSH/X11 forwarding. Não definir `DISPLAY=:0` por tentativa: usar
os valores reais herdados desse terminal. Informar o SHA completo publicado
junto desta entrega.

```bash
read -r -p 'SHA completo do novo commit desktop: ' desktop_commit
(
  set -euo pipefail
  [[ "$desktop_commit" =~ ^[0-9a-f]{40}$ ]]
  test "$desktop_commit" != 168a1fe69cbadb36c75458f799bf0dc3a6b5aacf
  task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
  desktop_root=$(mktemp -d "$task_root/desktop.XXXXXXXX")
  printf 'Pasta completa desta tentativa: %s\n' "$desktop_root"
  printf '%s\n' "$desktop_commit" > "$desktop_root/checkout_commit.txt"

  git -C "$task_root/gps_truck_nav" fetch origin \
    engineering/2026-10-preflight-gates \
    2>&1 | tee "$desktop_root/git_fetch.log"
  git -C "$task_root/gps_truck_nav" merge-base --is-ancestor "$desktop_commit" FETCH_HEAD
  git -C "$task_root/gps_truck_nav" merge-base --is-ancestor \
    168a1fe69cbadb36c75458f799bf0dc3a6b5aacf "$desktop_commit"
  GIT_LFS_SKIP_SMUDGE=1 git -C "$task_root/gps_truck_nav" worktree add \
    --detach "$desktop_root/gps_truck_nav" "$desktop_commit" \
    2>&1 | tee "$desktop_root/worktree.log"
  cd "$desktop_root/gps_truck_nav"
  git lfs checkout 2>&1 | tee "$desktop_root/lfs_checkout.log"
  git lfs fsck 2>&1 | tee "$desktop_root/lfs_fsck.log"
  test "$(git rev-parse HEAD)" = "$desktop_commit"
  test -z "$(git status --porcelain)"

  /usr/bin/python3 -I tools/auto_tuning/desktop_display.py diagnose \
    --output-dir "$desktop_root/display-diagnostic" \
    2>&1 | tee "$desktop_root/display-diagnostic-console.log"
  printf 'Diagnóstico concluído. Reutilizar esta pasta: %s\n' "$desktop_root"
)
```

Exigir `DISPLAY_DIAGNOSTIC_PASS` e limpeza da credencial confirmada. O JSON
registra tipo de sessão, display, presença de Wayland, caminho da autoridade,
verificação do acesso e GLX, sem gravar o segredo. O diagnóstico não inicia
Docker/ROS/Gazebo e não comprova ainda que as duas janelas renderizam.

Se falhar, preservar `display_diagnostic.json` e `display_guardian.log`. Não
contornar a falha com `xhost`, permissões abertas ou montagem da autoridade
original. Uma sessão Ubuntu on Xorg existente pode ser usada numa próxima
tentativa planejada, sem interromper tarefas em execução. Se não houver suporte
SECURITY adequado, a alternativa é um servidor X aninhado Xephyr com autenticação
própria: ele oferece desktop realmente interativo, mas exige uma configuração
separada, ainda não implementada nesta entrega. Não instalar ou trocar o display
automaticamente. O modo Xvfb continua disponível para os testes automatizados.

## 2. Build com cache, probe e regressões do modo padrão

A identidade da imagem muda porque há novos helpers e alterações nos
supervisores. É necessário construir a imagem deste commit; o Docker reutiliza
as camadas compatíveis. Não reutilizar o ID da imagem anterior como se contivesse
a correção. Não usar `--no-cache`, prune ou remoção de imagens/evidências.

```bash
read -r -p 'Pasta completa desktop desta tentativa: ' desktop_root
(
  set -euo pipefail
  test -f "$desktop_root/checkout_commit.txt"
  cd "$desktop_root/gps_truck_nav"
  test "$(git rev-parse HEAD)" = "$(cat "$desktop_root/checkout_commit.txt")"
  test -z "$(git status --porcelain)"

  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$desktop_root/gps_truck_nav" --output-dir "$desktop_root/build" \
    2>&1 | tee "$desktop_root/build-console.log"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
    --repo "$desktop_root/gps_truck_nav" --build-dir "$desktop_root/build" \
    --output-dir "$desktop_root/probe" --navigation-profile candidate20kmh \
    2>&1 | tee "$desktop_root/probe-console.log"

  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$desktop_root/gps_truck_nav" --build-dir "$desktop_root/build" \
    --probe-dir "$desktop_root/probe" --output-dir "$desktop_root/prepare-normal" \
    --gui-mode xvfb --fault-drill none --duration 45 --port 8765 \
    2>&1 | tee "$desktop_root/prepare-normal-console.log"
  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$desktop_root/gps_truck_nav" --build-dir "$desktop_root/build" \
    --probe-dir "$desktop_root/probe" --output-dir "$desktop_root/prepare-adapter" \
    --gui-mode xvfb --fault-drill adapter --duration 120 --port 8765 \
    2>&1 | tee "$desktop_root/prepare-adapter-console.log"
  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$desktop_root/gps_truck_nav" --build-dir "$desktop_root/build" \
    --probe-dir "$desktop_root/probe" --output-dir "$desktop_root/prepare-session" \
    --gui-mode xvfb --fault-drill session --duration 120 --port 8765 \
    2>&1 | tee "$desktop_root/prepare-session-console.log"

  /usr/bin/python3 -I tools/auto_tuning/check_r1_evidence.py \
    --root "$desktop_root" --output "$desktop_root/r1_evidence_review.json" \
    2>&1 | tee "$desktop_root/r1-review-console.log"
)
```

Exigir `IMAGE_VALIDATED`, probe `OBSERVATIONS_COLLECTED` e
`R1_READ_ONLY_VALIDATED` na nova imagem. Os templates falsos produzidos pelo modo
padrão preexistente não autorizam movimento; deixá-los intactos. A sequência
interrompe na primeira falha. Não continuar as etapas seguintes manualmente
para obter aprovação.

## 3. Abrir as duas janelas reais — somente observação

Executar no mesmo terminal local autorizado. Não abrir o navegador para este
modo. A sessão dura no máximo 180 s, mantendo o limite de preparação existente.

```bash
read -r -p 'Pasta completa desktop desta tentativa: ' desktop_root
(
  set -euo pipefail
  cd "$desktop_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$desktop_root/gps_truck_nav" --build-dir "$desktop_root/build" \
    --probe-dir "$desktop_root/probe" --output-dir "$desktop_root/desktop-normal" \
    --gui-mode desktop --fault-drill none --duration 180 \
    2>&1 | tee "$desktop_root/desktop-normal-console.log"
)
```

Quando o terminal confirmar as janelas, usar Alt-Tab para alternar Gazebo e RViz.
No RViz, selecionar a ferramenta de câmera, arrastar para mudar a vista e usar
a roda do mouse para zoom. No Gazebo, usar os controles de câmera para observar
o caminhão e o corredor. Não modificar o mundo nem usar comandos de navegação.
As duas janelas podem ser reposicionadas pelo gerenciador de janelas Ubuntu;
o monitor não as força de volta a uma posição fixa.

Deixar esta preparação terminar normalmente para validar também a finalização
R1. Fechar uma janela da aplicação é tratado como perda de um componente
supervisionado; para encerramento intencional antecipado, usar o cancelamento
abaixo. Uma tentativa cancelada preserva evidências, mas não aprova a conclusão
normal. O modo desktop não cria template de autorização operacional.

## 4. Terminal 2 — status e confirmação ao vivo de zero publishers

Executar enquanto a preparação estiver pronta e ativa. A leitura exige
heartbeat recente e identidade correspondente; não aceita uma evidência antiga
como prova de estado atual.

```bash
read -r -p 'Pasta completa desktop desta tentativa: ' desktop_root
(
  set -euo pipefail
  session_dir="$desktop_root/desktop-normal"
  test -f "$session_dir/manifest.json"
  /usr/bin/python3 -I "$desktop_root/gps_truck_nav/tools/auto_tuning/commissioning.py" status \
    --session-dir "$session_dir"
  /usr/bin/python3 -I - "$desktop_root/gps_truck_nav" "$session_dir" <<'PY'
import json
from datetime import datetime, timezone
from pathlib import Path
import sys

repo, session = map(Path, sys.argv[1:])
sys.path.insert(0, str(repo / 'tools/auto_tuning'))
import commissioning_supervision as safety

manifest = json.loads((session / 'manifest.json').read_text())
watch = safety.HeartbeatWatch(manifest['run_id'])
guard = safety.read_heartbeat(session / 'guard_heartbeat.json', watch)
assert manifest['mode'] == 'prepare' and manifest['gui_mode'] == 'desktop'
assert guard.get('mode') == 'prepare' and guard.get('gui_mode') == 'desktop'
assert guard.get('ready') is True and guard.get('gui_ready') is True
assert guard['adapter'].get('publisher_created') is False
assert guard['adapter'].get('observations_ready') is True
assert guard['adapter'].get('clock_sync_pending') is False
records = [json.loads(line) for line in (session / 'adapter/adapter_events.jsonl').read_text().splitlines()]
graph = next(record for record in reversed(records) if record.get('event') == 'GRAPH')
assert graph.get('valid') is True
assert graph.get('input_publishers') == 0 and graph.get('output_publishers') == 0
result = {'status': 'LIVE_READ_ONLY_ZERO_PUBLISHERS', 'run_id': manifest['run_id'],
          'guard': guard, 'last_graph_change': graph, 'movement_authorized': False}
stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
path = session / ('live_zero_publishers_' + stamp + '.json')
with path.open('x') as stream:
    json.dump(result, stream, indent=2, allow_nan=False)
    stream.write('\n')
print(json.dumps({'status': result['status'], 'evidence': str(path)}, indent=2))
PY
)
```

Se ainda estiver iniciando, aguardar a prontidão e repetir a leitura; cada
sucesso cria outro arquivo. A falha de leitura não deve ser interpretada como
confirmação de zero publishers.

## 5. Cancelar somente a sessão do ensaio

Este bloco é apenas para cancelamento antecipado. Não é necessário executá-lo
se a sessão terminar normalmente. O comando verifica CID, labels, imagem e
isolamento antes de encerrar o container pertencente à tentativa. Não usar
`docker stop` com curingas ou parar o daemon Docker.

```bash
read -r -p 'Pasta completa desktop desta tentativa: ' desktop_root
read -r -p 'Sessão ativa (desktop-normal, desktop-adapter ou desktop-session): ' desktop_session
(
  set -euo pipefail
  case "$desktop_session" in
    desktop-normal|desktop-adapter|desktop-session) ;;
    *) printf 'Nome de sessão inválido\n'; exit 1 ;;
  esac
  test -f "$desktop_root/$desktop_session/manifest.json"
  /usr/bin/python3 -I "$desktop_root/gps_truck_nav/tools/auto_tuning/commissioning.py" stop \
    --session-dir "$desktop_root/$desktop_session"
  /usr/bin/python3 -I "$desktop_root/gps_truck_nav/tools/auto_tuning/commissioning.py" status \
    --session-dir "$desktop_root/$desktop_session"
)
```

Ctrl-C no terminal supervisor também cancela. Exigir `Running=false`, `Pid=0`
e `OWNED_SIMULATOR_TERMINATED`. A limpeza da credencial gráfica é confirmada em
`display_cleanup.json` e no relatório da sessão. Se a revogação não for
confirmada, a tentativa falha mesmo que a planta já tenha sido contida.

## 6. Regressões adicionais do desktop antes de qualquer ensaio dinâmico

A aceitação de interação ocorre no passo 3. Antes de avaliar separadamente um
ensaio dinâmico futuro, repetir também as duas injeções no modo desktop: ele
altera os processos gráficos supervisionados e acrescenta o guardião de acesso.
Essas provas não exigem inspeção visual nem comandos de movimento.

```bash
read -r -p 'Pasta completa desktop desta tentativa: ' desktop_root
(
  set -euo pipefail
  cd "$desktop_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$desktop_root/gps_truck_nav" --build-dir "$desktop_root/build" \
    --probe-dir "$desktop_root/probe" --output-dir "$desktop_root/desktop-adapter" \
    --gui-mode desktop --fault-drill adapter --duration 120 \
    2>&1 | tee "$desktop_root/desktop-adapter-console.log"
  /usr/bin/python3 -I tools/auto_tuning/commissioning.py prepare \
    --repo "$desktop_root/gps_truck_nav" --build-dir "$desktop_root/build" \
    --probe-dir "$desktop_root/probe" --output-dir "$desktop_root/desktop-session" \
    --gui-mode desktop --fault-drill session --duration 120 \
    2>&1 | tee "$desktop_root/desktop-session-console.log"
)
```

Não renomear essas pastas para substituir as provas Xvfb. Guardar os seis
relatórios separadamente, vinculados ao mesmo commit e ID de imagem.

## Critérios de aceitação

| Evidência | Exigir |
|---|---|
| Display | `DISPLAY_DIAGNOSTIC_PASS`; sessão real X11/XWayland identificada; acesso e limpeza confirmados. |
| Build/probe | Nova imagem `IMAGE_VALIDATED`, perfil `candidate20kmh`, fonte e imagem coincidentes; coleta não equivale a aprovação integral do Nav2. |
| Modo padrão | As três preparações Xvfb aprovadas e `R1_READ_ONLY_VALIDATED` na imagem nova. |
| Desktop interativo | Gazebo e RViz como janelas reais; Alt-Tab, câmera e zoom funcionam; caminhão estacionário; monitor confirma janelas próprias. A confirmação de interação é do operador. |
| Desktop normal | `PREPARATION_COLLECTED`, `error=null`, `gui_mode=desktop`, `normal_telemetry_confirmed=true`; resumo da sessão com `desktop_windows_observed=true`. |
| Credencial gráfica | `display_cleanup.status=DESKTOP_ACCESS_REVOKED`, `guardian_returncode=0`, `authorization_revoked=true`, `credentials_removed=true`. |
| Autoridade/contenção | Zero publishers de atuação, nenhuma fonte de comando, `OWNED_SIMULATOR_TERMINATED`, `running=false`, `pid=0`. |
| Provas desktop | Ambas com `fault_drill_passed=true` e contenção inferior a 3 s, sem exigir finalização normal após SIGSTOP. |
| Próxima etapa | Gates 0/1 `NOT_YET_VERIFIED`; nenhuma autorização de movimento decorre destes resultados. |

As regressões offline verificam seleção do modo, autenticação/limpeza com
backends sintéticos, identificação de janelas e orquestração sem ROS/Docker.
Elas não comprovam acesso ao display do ipt-sim-01, funcionamento real GLX nem
interação de mouse/teclado. Esses itens dependem da execução acima. Os registros
offline publicados com o commit distinguem essa limitação.

Resultado offline: **668 testes PASS**, zero falhas, erros ou skips. Foram
adicionadas 74 regressões: 27 de autenticação/limpeza, 23 do monitor de janelas e
24 da integração desktop. Incluem as duas contenções simuladas, falha de acesso
antes da criação do container, revogação depois da contenção, erro de saída do
guardião, troca de workspace e fechamento concorrente de janela alheia. Os sete
blocos Bash e os dois trechos Python deste documento passaram nas verificações
de sintaxe. Evidências:
[`desktop_unit_tests.log`](evidence/desktop_unit_tests.log) e
[`desktop_offline_summary.json`](evidence/desktop_offline_summary.json).

## Preservar e encaminhar evidências

Após as sessões terminarem, o bloco cria arquivo novo e inventário SHA-256,
inclusive para tentativas incompletas. Não segue o caminho da credencial gráfica
registrado no JSON e não inclui `XDG_RUNTIME_DIR`, a autoridade do usuário ou
segredos X11. As pastas e imagens anteriores permanecem intactas.

```bash
read -r -p 'Pasta completa desktop a consolidar: ' desktop_root
(
  set -euo pipefail
  test -f "$desktop_root/checkout_commit.txt"
  cd "$desktop_root"
  /usr/bin/python3 -I - <<'PY'
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import tarfile

folders = ('display-diagnostic', 'build', 'probe', 'prepare-normal', 'prepare-adapter',
           'prepare-session', 'desktop-normal', 'desktop-adapter', 'desktop-session')
files = []
for folder in folders:
    for path in sorted(Path(folder).rglob('*')):
        if path.is_file() and not path.is_symlink():
            files.append(path)
files += sorted(Path('.').glob('*.log'))
files += [Path('checkout_commit.txt')]
if Path('r1_evidence_review.json').is_file():
    files.append(Path('r1_evidence_review.json'))
stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
inventory = Path('desktop_inventory_' + stamp + '.json')
records = []
for path in files:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            digest.update(block)
    records.append({'path': str(path), 'bytes': path.stat().st_size, 'sha256': digest.hexdigest()})
with inventory.open('x') as stream:
    json.dump(records, stream, indent=2)
    stream.write('\n')
archive = Path('arocs_desktop_review_' + stamp + '.tar.gz')
with tarfile.open(archive, 'x:gz') as bundle:
    for path in files + [inventory]:
        bundle.add(path, recursive=False)
digest = hashlib.sha256()
with archive.open('rb') as stream:
    for block in iter(lambda: stream.read(1048576), b''):
        digest.update(block)
print('Arquivo:', archive.resolve())
print('SHA-256:', digest.hexdigest())
PY
)
```

Encaminhar o arquivo, SHA-256 e a confirmação humana de interação com as duas
janelas. Não executar `commissioning.py execute`, criar/preencher autorização ou
iniciar movimento nesta etapa.
