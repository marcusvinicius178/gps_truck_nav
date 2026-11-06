# Malhas ausentes no Gazebo — correção do cliente gráfico

O retorno do ipt-sim-01 em 06/11/2026, commit 883fe4b, confirma IMAGE_VALIDATED,
OBSERVATIONS_COLLECTED e evidências shadow legíveis como marcus:marcus 0700/0600
(UID/GID 1001). SHADOW_COMPLETE, NOT_EXERCISED, zero publishers de atuação. A
captura mostra o Arocs completo no RViz e apenas elementos primitivos no Gazebo.
O diagnóstico Nav2 continua INCOMPLETE/PARTIAL e mission_ready=false; esta
correção não reclassifica a prontidão nem investiga novos bloqueios de navegação.

## Causa e correção limitada

O SDF referencia malhas como model://arocs_truck/meshes/cabine_chassi.dae e o
material do terreno usa model://ctvi/materials/.... Esses assets são instalados
em /ws/install/truck_bringup/share/truck_bringup/models.

O launch acrescenta esse diretório ao ambiente do gzserver. O supervisor da
visualização inicia gzclient separadamente; as alterações de ambiente de um
processo filho não retornam ao supervisor nem alcançam seus outros filhos.
O cliente recebia somente os modelos prospectivos/config e a cópia gráfica,
sem as malhas canônicas e os recursos CTVI. RViz usa package://truck_bringup/...
e os encontra pelo índice ament, explicando a diferença visível.

A sessão agora resolve o share instalado pelo ament e entrega os caminhos
GAZEBO_MODEL_PATH e GAZEBO_RESOURCE_PATH a todos os processos que inicia,
incluindo gzclient. Mantém a cópia de apresentação à frente dos aliases config.
Antes de abrir as janelas, resolve as malhas referenciadas e os arquivos de
material/textura CTVI com os mesmos caminhos. Ausência, arquivo ilegível ou
ponteiro LFS produzem erro explícito. O presentation_manifest.json registra os
caminhos completos e visual_assets com URI, arquivo resolvido e tamanho.

Isso comprova resolução/leitura local, não renderização OGRE. O log anexado não
continha gazebo_gui.log; a causa foi identificada no código e reproduzida offline
com o SDF real e assets sintéticos. A nova renderização ainda depende do host.
Nenhuma câmera, sensor, flag visualize, RViz, URDF, geometria, limite, parâmetro
Nav2 ou interface de atuação é alterado neste incremento.

Fonte oficial: [SystemPaths.cc Classic 11.10.2](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/gazebo/common/SystemPaths.cc),
blob `2e0f7704339fdd8435bce978507011717900dca3`, UpdateModelPaths e FindFileURI.
A instalação local é definida em CMakeLists.txt e o ambiente do servidor em
truck_simulation_launch.py. Os arquivos físicos permanecem preservados.

## 1. Encerrar a tentativa recebida, se ainda estiver aberta

Se o terminal estiver parado no prompt de read, digitar somente:

```text
/home/marcus/nav2-phase-a-ipt-sim-01.pMbd0n06/visual-fix.R15FaYCG
```

Num prompt normal, este bloco dispensa a pergunta e usa o caminho exato do log:

```bash
visual_fix_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06/visual-fix.R15FaYCG"
/usr/bin/python3 -I "$visual_fix_root/gps_truck_nav/tools/auto_tuning/visual_observer.py" status \
  --session-dir "$visual_fix_root/visual"
/usr/bin/python3 -I "$visual_fix_root/gps_truck_nav/tools/auto_tuning/visual_observer.py" stop \
  --session-dir "$visual_fix_root/visual"
```

Não apagar ou editar essa tentativa. Seu sucesso de permissões e sua falha
visual permanecem registrados. O stop verifica a propriedade do container.

## 2. Novo build com cache e um probe do mesmo candidato

No ipt-sim-01, executar o bloco completo. O SHA exato da entrega consta na
resposta; pode substituir FETCH_HEAD no worktree add por esse SHA.

```bash
task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
mesh_fix_root=$(mktemp -d "$task_root/mesh-fix.XXXXXXXX")
printf 'Novas evidências: %s\n' "$mesh_fix_root"
(
  set -euo pipefail
  git -C "$task_root/gps_truck_nav" fetch origin engineering/2026-10-preflight-gates \
    2>&1 | tee "$mesh_fix_root/git_fetch.log"
  GIT_LFS_SKIP_SMUDGE=1 git -C "$task_root/gps_truck_nav" worktree add --detach \
    "$mesh_fix_root/gps_truck_nav" FETCH_HEAD
  cd "$mesh_fix_root/gps_truck_nav"
  git lfs checkout 2>&1 | tee "$mesh_fix_root/lfs_checkout.log"
  git lfs fsck 2>&1 | tee "$mesh_fix_root/lfs_fsck.log"
  git rev-parse HEAD > "$mesh_fix_root/checkout_commit.txt"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$mesh_fix_root/gps_truck_nav" --output-dir "$mesh_fix_root/build"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
    --repo "$mesh_fix_root/gps_truck_nav" --build-dir "$mesh_fix_root/build" \
    --output-dir "$mesh_fix_root/probe" --navigation-profile candidate20kmh
)
```

Dockerfile, APT, colcon e modelos não mudaram; as camadas continuam elegíveis ao
cache se base/cache forem iguais. Não usar prune, --no-cache, reinício Docker ou
instalações no host. A nova imagem precisa de seu probe para preservar o vínculo
fonte/imagem/evidência; o helper recusa misturar o probe antigo com outra fonte.

## 3. Abrir e conferir Gazebo/RViz

No mesmo terminal:

```bash
(
  set -euo pipefail
  cd "$mesh_fix_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py start \
    --repo "$mesh_fix_root/gps_truck_nav" --build-dir "$mesh_fix_root/build" \
    --probe-dir "$mesh_fix_root/probe" --output-dir "$mesh_fix_root/visual" --duration 900
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py serve \
    --session-dir "$mesh_fix_root/visual" --port 8765 --capture-frames 3
)
```

Abrir no navegador local a URL impressa. Conferir cabine/chassi/rodas no Gazebo,
terreno com sua textura e RViz preservado. A existência de janelas e arquivos
não substitui essa inspeção. As três capturas e os logs ficam na pasta nova.

Ctrl-C no serve encerra o container próprio. Para encerrar por outro terminal,
usar a pasta mesh-fix impressa acima, ou manter a variável no terminal atual:

```bash
/usr/bin/python3 -I "$mesh_fix_root/gps_truck_nav/tools/auto_tuning/visual_observer.py" stop \
  --session-dir "$mesh_fix_root/visual"
```

Se ainda faltar uma malha, retornar visual/gazebo_gui.log,
visual/presentation_manifest.json e a nova captura. O manifest precisa mostrar
as URIs Arocs/CTVI resolvidas no share instalado. Não modificar a geometria ou
substituir malhas para mascarar falha de resolução.

Sem goals, Twist, movimento ou saída conectada do adaptador. PR #2 permanece
DRAFT; Gates 0/1 permanecem NOT_YET_VERIFIED.

Resultado offline: **429 testes PASS**, incluindo quatro regressões novas de
resolução e um subprocesso independente; auditoria estrutural PASS; 32 blocos
Bash/9 Python embutidos com sintaxe verificada; 31 arquivos protegidos iguais à
base. Evidências em `evidence/gazebo_assets_fix_tests.log`,
`evidence/gazebo_assets_fix_checks.log` e `evidence/gazebo_assets_fix_summary.json`.
