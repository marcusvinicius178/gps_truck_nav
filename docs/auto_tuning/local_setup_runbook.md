# Preparação local — ipt-sim-01, continuação da Fase A

**HOST PREFLIGHT APROVADO pelo revisor. Gates 0 e 1: NOT_YET_VERIFIED.** O ZIP do commit `a21403f` confirma TF/frames, odometria/juntas, costmaps, duas BTs, lifecycle e parâmetros obrigatórios; esses reparos permanecem preservados. Seguir agora a **seção 2.5**: novo checkout isolado, build, probe do candidato Classic e visualização estritamente observacional. A variante ainda não é uma planta dinamicamente validada. Nenhum movimento. As seções anteriores ficam como histórico operacional.

## 1. Ambiente confirmado e isolamento

O `host_preflight.json` coletado em 29/10/2026 às 16:00:40 (São Paulo) foi lido integralmente. Ele confirma Ubuntu 22.04.5 x86_64, Python 3.10.12, 32 CPUs lógicas, 60,3 GiB RAM (40,8 GiB disponíveis naquele instante), Docker Engine 29.8.2, Compose 5.5.1 e daemon em `unix:///var/run/docker.sock`. Git/LFS estão presentes. ROS 2 Humble existe no host; Gazebo não está no PATH do host. Esses fatos são compatíveis com a preparação pelo Docker.

Os 366,6 GiB livres pertencem ao filesystem de Downloads. O helper mede separadamente o diretório de evidências e DockerRootDir; Docker 29 também pode usar image store containerd, com caminho padrão `/var/lib/containerd` separado. Quando acessível, esse caminho é medido como candidato padrão, não como prova de que não há uma configuração customizada. Falta de acesso fica registrada como desconhecida. `docker system df` não substitui espaço livre do filesystem.

Não há instalação ou atualização de Docker, ROS, drivers, pacotes apt/pip, ambientes Python ou shell no host. Os comandos usam `/usr/bin/python3 -I` e stdlib. Humble, CARLA e Autoware permanecem em seus diretórios e containers atuais. O workspace desta atividade será novo e não terá mounts de nenhum ambiente existente.

O helper fixa o daemon local e o builder `default` nos subprocessos, neutralizando overrides de contexto/builder sem editar a configuração global. Runtime usa imagem por ID, rede `none`, sem portas publicadas, host networking, GPU, dispositivos, X11, privilégios ou socket Docker montados. O probe recebe quota de 8 CPUs, limite de 16 GiB, sem swap adicional, 2048 PIDs e 1 GiB de shared memory. Essas quotas não reservam CPUs nem garantem desempenho de aplicações concorrentes. O build limita CMake/Make a quatro jobs e colcon a um pacote por vez; isso **não** é um teto global de CPU/RAM para BuildKit/APT.

Renderização inicial: Xvfb+xauth+Mesa na imagem, `LIBGL_ALWAYS_SOFTWARE=1`. É uma proposta para diagnosticar os sensores sem compartilhar a GPU; funcionamento de câmera/OpenGL no Gazebo ainda precisa ser observado. Não desabilitar sensores ou alterar o mundo para obter um resultado aparente.

## 2. Agora: checkout novo, modelos e build

**Diretório inicial:** `$HOME`. **Pré-requisitos já confirmados:** Git, Git LFS, Python 3 e acesso ao Docker local sem sudo. É necessária conexão para GitHub, registry e APT dentro do build. Não executar `source /opt/ros/...` no host.

Execute o bloco completo em um terminal do `ipt-sim-01`. Ele cria um diretório exclusivo e mantém `task_root` no terminal para os blocos seguintes. Em caso de erro, os passos posteriores dentro dos parênteses não são executados; os logs já gravados permanecem.

```bash
cd "$HOME"
task_root=$(mktemp -d "$HOME/nav2-phase-a-ipt-sim-01.XXXXXXXX")
printf 'Diretório desta execução: %s\n' "$task_root"
(
  set -euo pipefail
  GIT_LFS_SKIP_SMUDGE=1 git clone \
    --branch engineering/2026-10-preflight-gates \
    https://github.com/marcusvinicius178/gps_truck_nav.git \
    "$task_root/gps_truck_nav"
  cd "$task_root/gps_truck_nav"
  git lfs install --local
  git lfs pull 2>&1 | tee "$task_root/lfs_pull.log"
  git lfs fsck 2>&1 | tee "$task_root/lfs_fsck.log"
  git rev-parse HEAD > "$task_root/checkout_commit.txt"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$task_root/gps_truck_nav" \
    --output-dir "$task_root/build"
)
```

Nenhum arquivo de código precisa ser montado por trechos ou inserido manualmente. Os arquivos completos estão no branch da PR #2. Não é preciso clonar o repositório de análise nesta etapa sem missão. Não atualizar esse checkout depois do build: o helper vincula imagem, commit, tree e hashes dos inputs; o probe recusa uma fonte diferente. Arquivos de evidência ficam fora do checkout para que sua integridade possa ser verificada.

O estágio `build`:

1. Confere fonte limpa, assets LFS, daemon/builder locais e registra as capacidades de disco observáveis.
2. Faz pull de `osrf/ros:iron-desktop-full` para `linux/amd64` e resolve o RepoDigest real.
3. Constrói a receita `docker/auto_tuning/Dockerfile` usando esse digest, sem tag de imagem compartilhada ou volumes antigos. Compila somente `truck_bringup` contra Nav2 binário Iron; não executa `vcs import`.
4. Registra image ID/inspect e lista de pacotes, e executa `validate_image.sh` em um container próprio sem rede externa.
5. Confere Ubuntu 22.04, Iron, Gazebo 11, imports, registro reader/writer MCAP, prefixos de pacotes, biblioteca local e suas dependências, declaração plugin XML e parsing de `ros2 launch --show-args`.
6. Preserva logs/estado e finaliza somente o container criado, inclusive em falha/timeout ou primeira interrupção durante o trabalho. Interrupção durante o próprio cleanup pode exigir a recuperação da seção 4. Não remove containers/imagens de outros projetos nem executa prune.

`IMAGE_VALIDATED` em `build/status.json` significa build e smoke checks da imagem concluídos. Não significa que Gazebo/Nav2 iniciaram ou que pluginlib, sensores, TF e navegação funcionam. Erro de rede/APT/build/checagem fica no status e nos logs; não trocar Iron, remover verificação de assinatura ou instalar dependências no host para contornar uma falha.

O helper imprime o caminho de cada log; a saída extensa de pull/build fica nesses arquivos. Para acompanhar o build em outro terminal, atribuir o caminho impresso a `task_root` e executar `tail -f "$task_root/build/build.log"` depois que o arquivo existir. Interromper apenas o `tail` não interrompe o build.

**Primeiro ponto de retorno:** se o build falhar, envie `build/status.json`, `build/commands.jsonl` e os logs indicados. Não executar o probe com uma imagem diferente escolhida manualmente.

### 2.1. Repetir após a correção de `gzserver --version`

O proprietário informou que a imagem do commit `6af48f2` foi construída e que apenas o check `gazebo_version` falhou: banner Classic 11.10.2 correto, retorno 255. No source upstream dessa versão, `Server::ParseArgs()` imprime a versão e retorna false; `server_main.cc` retorna -1 antes de `server->Run()`. No Linux, o status observado é 255. Isso explica a consulta de versão; não comprova execução funcional do simulador.

O validador agora aceita essa combinação exclusivamente em `gzserver --version`: banner completo Classic 11 com versão numérica inequívoca, término sem timeout e retorno 0 ou 255. Mantém `returncode`, stdout/stderr, `process_succeeded` e `accepted_nonzero_exit` no JSON. `validation_scope=VERSION_IDENTIFICATION_ONLY` e `simulator_runtime_verified=false` deixam o limite explícito. Outros comandos continuam exigindo exit zero. Saída ausente/inválida/conflitante, outro major com 255, outros códigos/sinais e timeout continuam reprovando.

**Diretório de trabalho:** o checkout dentro do `task_root` anterior, sem alterações locais. Este bloco busca a revisão da PR, preserva o diretório `build` anterior e cria uma tentativa exclusiva. Se ainda estiver no mesmo terminal, `task_root` já está definido; caso contrário, o comando solicita o caminho completo impresso na execução anterior. Não selecionar um ambiente CARLA/Autoware.

```bash
if [[ -z "${task_root:-}" ]]; then
  read -r -p 'Caminho completo do diretório nav2-phase-a-ipt-sim-01 anterior: ' task_root
fi
retry_root=$(mktemp -d "$task_root/smoke-retry.XXXXXXXX")
printf 'Novas evidências: %s\n' "$retry_root"
(
  set -euo pipefail
  test -n "$retry_root"
  test -d "$retry_root"
  test -f "$task_root/build/status.json"
  cd "$task_root/gps_truck_nav"
  test -z "$(git status --porcelain --untracked-files=all)"
  git fetch origin engineering/2026-10-preflight-gates \
    2>&1 | tee "$retry_root/git_fetch.log"
  GIT_LFS_SKIP_SMUDGE=1 git checkout --detach FETCH_HEAD
  git rev-parse HEAD > "$retry_root/checkout_commit.txt"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$task_root/gps_truck_nav" \
    --output-dir "$retry_root/build"
)
```

Não usar `--no-cache`, prune ou apagar a imagem anterior. Como o Dockerfile e os inputs de APT/C++ não mudam nesta correção, essas camadas podem aparecer como `CACHED` em `build/build.log` se o digest da base e o cache local continuarem iguais. A mudança é copiada depois do build C++; somente a parte final da imagem precisa ser atualizada nesse caso. O helper ainda faz pull da base e registra seu digest: se ele mudar ou o cache não existir, não há garantia de reutilização. Um novo image ID é esperado; não editar o status anterior para transformá-lo em sucesso.

Retornar `retry_root/build/status.json`, `image_validation_run.log`, `build.log` e `commands.jsonl` (o caminho real de `retry_root` é impresso acima). O JSON do smoke continua usando `IMAGE_SMOKE_VALIDATED`; o helper só grava `IMAGE_VALIDATED` após a checagem, cópia do manifesto e cleanup completos. O retorno 255 deve permanecer no registro `gazebo_version`, agora com `process_succeeded=false` e `accepted_nonzero_exit=true` quando identificado corretamente.

**Parar aqui para revisar o novo resultado.** Não executar o bringup desta tentativa antes de `IMAGE_VALIDATED`. Quando avançarmos à seção 3, usar `--build-dir "$retry_root/build"` e um novo `--output-dir "$retry_root/probe"`, mantendo o mesmo checkout congelado. Para empacotar somente esta tentativa pelo bloco da seção 4, passar `"$retry_root"` como argumento do Python em vez de `"$task_root"`. Gates 0/1 continuam não verificados; todos os bloqueios de missão permanecem.

Fontes da correção: [Server::ParseArgs, Classic 11.10.2](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/gazebo/Server.cc), [main de gzserver](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/gazebo/server_main.cc).

### 2.2. Revisão independente: costmaps, BTs e prontidão objetiva

Ler `independent_bringup_review.md` e `localization_actuation_bringup_review.md`. Foram corrigidos o transporte do YAML completo até o processo container e a seleção prospectiva de duas árvores BT sem Spin/BackUp. A localização e a interface de atuação continuam bloqueadas para missão. A nova coleta mede essas pendências; não muda datum, geometria, covariâncias nem habilita Virtual Lanes.

**Host:** ipt-sim-01. **Diretório inicial:** qualquer diretório do terminal desse host. O caminho abaixo foi confirmado no ZIP recebido. Se o proprietário tiver movido esse diretório, usar seu novo caminho exato, sem selecionar um workspace diferente. O checkout deve estar limpo; não descartar mudanças locais para passar nessa checagem.

```bash
task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
review_root=$(mktemp -d "$task_root/bringup-review.XXXXXXXX")
printf 'Evidências desta revisão: %s\n' "$review_root"
(
  set -euo pipefail
  test -n "$review_root"
  test -d "$review_root"
  cd "$task_root/gps_truck_nav"
  test -z "$(git status --porcelain --untracked-files=all)"
  git fetch origin engineering/2026-10-preflight-gates \
    2>&1 | tee "$review_root/git_fetch.log"
  GIT_LFS_SKIP_SMUDGE=1 git checkout --detach FETCH_HEAD
  git rev-parse HEAD > "$review_root/checkout_commit.txt"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$task_root/gps_truck_nav" \
    --output-dir "$review_root/build"
)
```

Esse build mantém o daemon/builder locais e pode reutilizar APT/base em cache. O launch mudou dentro do pacote: o colcon pode reconstruir/reinstalar essa camada. Também é compilado o novo observador TF em C++, dentro da imagem, com dois jobs. Não usar `--no-cache`, prune, reinício do Docker ou instalação no host. O smoke verifica o transporte dos parâmetros usando o launch Iron instalado, dependências do observador e os checks anteriores; isso ainda não demonstra parâmetros aplicados em nós vivos.

**Somente após `IMAGE_VALIDATED` nesse novo build**, no mesmo terminal e sem atualizar o checkout novamente, executar:

```bash
cd "$task_root/gps_truck_nav"
/usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
  --repo "$task_root/gps_truck_nav" \
  --build-dir "$review_root/build" \
  --output-dir "$review_root/probe"
```

O helper seleciona `use_ackermann_bt:=true`, o perfil prospectivo de localização anterior e `publish_legacy_cloud_tf:=false`. Não envia goals/Twist, não grava bag, não inicia otimizador e não aciona Virtual Lanes. O observador adiciona uma janela limitada para parâmetros/lifecycle, amostras de relógio/sensores, metadados das costmaps e TF por aresta/GID. `/fromLL` recebe apenas consultas de conversão; nenhum serviço de reset, datum ou atuação é chamado.

Interpretar os resultados separadamente:

| Campo | Significado |
|---|---|
| `status=OBSERVATIONS_COLLECTED` | Coleta retornou e container próprio foi encerrado; não é saúde integral. |
| `readiness_collection_status` | COMPLETE/PARTIAL/FAILED da captura estruturada. |
| `diagnostic_status=BLOCKED` | Há critérios objetivos reprovados. |
| `diagnostic_status=INCOMPLETE` | Evidência necessária ausente/inconclusiva. |
| `diagnostic_status=REVIEW_REQUIRED` | Os checks observados não liberam missão; revisão geométrica/atuação continua necessária. |
| `mission_ready=false` | Mantido em todos os resultados. Gates 0/1 continuam não verificados. |

Revisar `probe/runtime_probe/readiness.json` junto de `readiness_observations.json`, `tf_authority.json`, dumps de parâmetros, estados lifecycle e `probe/bringup.log`. Para costmaps, conferir dimensões, rolling window, footprint, frames e camadas/fontes contra o YAML, incluindo ausência de StaticLayer. Para BT, conferir os dois navigators/path XML e todos os sete lifecycle nodes ACTIVE. Falhas de fontes de obstáculos, covariâncias, frames ou autoridade não devem ser ocultadas por um container ainda vivo.

Os metadados GPS/IMU antigos e a interface Ackermann ainda podem produzir BLOCKED mesmo após a correção de costmaps/BT. Isso é evidência útil, não autorização para escolher novos valores de ruído ou geometria. Timeout ou falha do observador devem aparecer explicitamente; o helper recusa relatório ausente/inválido e preserva cleanup. Interrupção durante o próprio cleanup requer a recuperação por ID/labels da seção 4, usando `stage_dir="$review_root/probe"`.

Para empacotar esta tentativa sem sobrescrever pacotes anteriores:

```bash
/usr/bin/python3 -I - "$review_root" <<'PY'
from pathlib import Path
import sys
import zipfile
root = Path(sys.argv[1]).resolve(strict=True)
archive = root / 'nav2_phase_a_review_evidence.zip'
allowed = {'.json', '.jsonl', '.log', '.txt', '.tsv', '.yaml'}
with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as output:
    for name in ('checkout_commit.txt', 'git_fetch.log'):
        path = root / name
        if path.is_file() and not path.is_symlink():
            output.write(path, path.relative_to(root))
    for name in ('build', 'probe'):
        directory = root / name
        if not directory.is_dir() or directory.is_symlink():
            continue
        for path in sorted(directory.rglob('*')):
            if (path.is_file() and not path.is_symlink()
                    and path.resolve().is_relative_to(directory.resolve())
                    and (path.suffix in allowed or path.name == 'container.id')):
                output.write(path, path.relative_to(root))
print(archive)
PY
```

Encaminhar esse ZIP ao revisor independente. Não iniciar missão após o probe, mesmo se todos os lifecycle nodes ficarem ACTIVE. As tentativas anteriores em `build` e `smoke-retry.N5Na75vV` permanecem intactas; a revisão atual usa somente o novo diretório `bringup-review.*`. Não editar manifests antigos, não remover containers/imagens e não fazer merge da PR #2.

### 2.3. Segunda revisão: TF, sensores e evidência dos plugins

Ler `second_independent_review.md`, `localization_covariance_policy.md`, `ackermann_interface_contract.md` e `plugin_parameter_observability.md`. O SDF prospectivo completo tem exatamente três mudanças: desabilita `publish_wheel_tf` e define os frames GPS/IMU. O mundo prospectivo muda apenas a URI desse modelo. O modelo/URDF/YAML históricos, datum, geometria, covariâncias, BTs e launches permanecem iguais. Não há conversor de atuação conectado.

**Host:** ipt-sim-01. **Diretório inicial:** qualquer diretório do terminal desse host. Os comandos usam o checkout dedicado já confirmado nas evidências; exigem que esteja limpo e preservam todas as tentativas anteriores. Executar o bloco inteiro. Ele cria uma nova pasta exclusiva e grava o commit efetivamente selecionado; a resposta de entrega também fornece o SHA fixo para esta revisão.

```bash
task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
review2_root=$(mktemp -d "$task_root/second-review.XXXXXXXX")
printf 'Evidências desta revisão: %s\n' "$review2_root"
(
  set -euo pipefail
  test -n "$review2_root"
  test -d "$review2_root"
  cd "$task_root/gps_truck_nav"
  test -z "$(git status --porcelain --untracked-files=all)"
  git fetch origin engineering/2026-10-preflight-gates \
    2>&1 | tee "$review2_root/git_fetch.log"
  GIT_LFS_SKIP_SMUDGE=1 git checkout --detach FETCH_HEAD
  test -f config/auto_tuning/tf_sensor_model_variant.json
  git rev-parse HEAD > "$review2_root/checkout_commit.txt"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$task_root/gps_truck_nav" \
    --output-dir "$review2_root/build"
)
```

O Docker pode reutilizar as camadas APT e colcon: seus inputs não foram alterados nesta revisão. A variante/configuração e os coletores são copiados depois dessas camadas; o observador C++ é recompilado na imagem. Reutilização depende do cache e do digest de base resolvido, ambos registrados. Não usar `--no-cache`, prune, remoção de imagens, reinstalação/reinício Docker ou `source` de ROS no host. O smoke agora verifica também o manifest e os assets instalados da variante. `gazebo_version` continua preservando 255 como evidência de identificação de versão, sem alegar execução funcional.

**Somente após IMAGE_VALIDATED no build acima**, no mesmo terminal, executar o bloco completo abaixo. Ele confere explicitamente o status, não atualiza o checkout e usa a imagem pelo ID registrado:

```bash
(
  set -euo pipefail
  cd "$task_root/gps_truck_nav"
  /usr/bin/python3 -I - "$review2_root/build/status.json" <<'PY'
import json
import sys
status = json.load(open(sys.argv[1]))
if status.get('status') != 'IMAGE_VALIDATED':
    raise SystemExit('Build não está IMAGE_VALIDATED; preserve os logs e pare aqui.')
PY
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
    --repo "$task_root/gps_truck_nav" \
    --build-dir "$review2_root/build" \
    --output-dir "$review2_root/probe"
)
```

O helper mantém `use_ackermann_bt:=true`, perfil de localização anterior e `publish_legacy_cloud_tf:=false`; seleciona explicitamente `world:=/opt/auto_tuning/config/worlds/ctvi_tf_sensors.world`. Registra um snapshot `bringup_before_probe.log` para evidência interna dos plugins e coleta apenas observações/consultas de leitura. A captura `/proc` é feita somente dentro do container próprio, sem acesso ao `/proc` do host. Os endpoints de `/cmd_vel` são consultados no grafo; nenhum Twist ou goal é enviado. Não há bag novo, missão, otimizador ou Virtual Lanes.

| Evidência nova em `probe/runtime_probe/` | O que examinar |
|---|---|
| `tf_authority.json` schema 2 | Clock na mesma janela, timestamps/recepções por aresta e GID, QoS/perdas quando disponíveis, seis pais URDF e autoridade única. |
| `plugin_parameters.json` | Tentativas Get/Describe, nomes/tipos/valores efetivos, deadlines, REQUIRED/optional, versões/lib/plugin XML e distinção DECLARED/EFFECTIVE/ACTUALLY_USED. |
| `runtime_binary_identity.json` | Bibliotecas realmente mapeadas, executáveis, device/inode/hash e comparação com arquivos instalados; não prova equivalência fonte-binário. |
| `localization_alignment.json` | Fix, odometria GPS e três TF no mesmo stamp inteiro, rotação 3D dos lever arms e residuais horizontais; ausência de amostra exata não vira latest. |
| `ackermann_geometry.json` | Cálculo estático e conflitos de limites; `runtime_geometry_validated=false`, `motion_authorized=false`. |
| `readiness_observations.json` e `readiness.json` | Evidência bruta, checks, captura COMPLETE/PARTIAL/FAILED e diagnóstico separado da aprovação de missão. |

Os dumps CLI anteriores continuam sendo coletados. Um dump vazio com exit zero não aprova os parâmetros. `plugin_parameters.required_collection_status=COMPLETE` indica captura Get/Describe de todos os obrigatórios; `collection_status=PARTIAL` pode resultar de opcionais não declarados ou de timeout, com causa explícita por tentativa. Nenhum desses campos comprova consumo algorítmico. Costmaps/BTs devem manter os resultados já confirmados. Covariâncias nulas/desconhecidas, fonte externa ausente, atuação e precisão geométrica continuam pendentes mesmo com frames/TF corrigidos.

Para empacotar **somente esta nova tentativa**, inclusive falhas, sem sobrescrever ZIPs anteriores, executar:

```bash
/usr/bin/python3 -I - "$review2_root" <<'PY'
from pathlib import Path
import hashlib
import json
import sys
import zipfile
root = Path(sys.argv[1]).resolve(strict=True)
archive = root / 'nav2_phase_a_second_review_evidence.zip'
allowed = {'.json', '.jsonl', '.log', '.txt', '.tsv', '.yaml'}
with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as output:
    for name in ('checkout_commit.txt', 'git_fetch.log'):
        path = root / name
        if path.is_file() and not path.is_symlink():
            output.write(path, path.relative_to(root))
    for name in ('build', 'probe'):
        directory = root / name
        if not directory.is_dir() or directory.is_symlink():
            continue
        for path in sorted(directory.rglob('*')):
            if (path.is_file() and not path.is_symlink()
                    and path.resolve().is_relative_to(directory.resolve())
                    and (path.suffix in allowed or path.name == 'container.id')):
                output.write(path, path.relative_to(root))
digest = hashlib.sha256(archive.read_bytes()).hexdigest()
with (root / 'archive_identity.json').open('x') as stream:
    json.dump({'archive': archive.name, 'sha256': digest}, stream, indent=2)
    stream.write('\n')
print(archive)
print('SHA-256:', digest)
PY
```

Encaminhar o ZIP e o SHA-256 ao revisor independente. As pastas `build`, `smoke-retry.*` e `bringup-review.ZDIBJp1z` anteriores ficam intactas, assim como as imagens e containers parados. Se o cleanup desta tentativa for interrompido, usar o bloco completo da seção 4 com `stage_dir="$review2_root/probe"` (ou `build` para esse estágio), conferindo ID e labels. Não iniciar outra coleta com o mesmo output nem editar os manifests para reaproveitar imagem incompatível. A próxima decisão é do revisor; **OBSERVATIONS_COLLECTED não libera missão.**

### 2.4. Terceira revisão: instrumentação, perfis e visualização

Ler `third_independent_review.md`, `speed_limits_provenance.md`, `actuation_validation_protocol.md` e `visual_observation_runbook.md`. O MPPI 25 m/s não foi tratado como typo de 2,5. Os perfis experimentais completos estão separados do histórico e ainda bloqueados para atuação. O primeiro probe abaixo mantém `canonical` para isolar as correções de instrumentação.

**Host:** ipt-sim-01. **Diretório inicial:** qualquer diretório desse host. **Pré-requisito:** checkout dedicado limpo, sem descartar alterações locais. Executar o bloco completo em Bash; o SHA exato desta entrega também consta na resposta de entrega. A pasta nova preserva todas as imagens, containers parados e evidências anteriores.

```bash
task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
review3_root=$(mktemp -d "$task_root/third-review.XXXXXXXX")
printf 'Evidências desta revisão: %s\n' "$review3_root"
(
  set -euo pipefail
  test -n "$review3_root"
  test -d "$review3_root"
  cd "$task_root/gps_truck_nav"
  test -z "$(git status --porcelain --untracked-files=all)"
  git fetch origin engineering/2026-10-preflight-gates \
    2>&1 | tee "$review3_root/git_fetch.log"
  GIT_LFS_SKIP_SMUDGE=1 git checkout --detach FETCH_HEAD
  test -f tools/auto_tuning/visual_observer.py
  test -f config/auto_tuning/navigation_profiles/manifest.json
  git rev-parse HEAD > "$review3_root/checkout_commit.txt"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$task_root/gps_truck_nav" \
    --output-dir "$review3_root/build"
  /usr/bin/python3 -I - "$review3_root/build/status.json" <<'PY'
import json
import sys
if json.load(open(sys.argv[1])).get('status') != 'IMAGE_VALIDATED':
    raise SystemExit('Imagem não validada; preserve os logs e não execute o probe.')
PY
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
    --repo "$task_root/gps_truck_nav" \
    --build-dir "$review3_root/build" \
    --output-dir "$review3_root/probe" \
    --navigation-profile canonical
)
```

Não usar `--no-cache`, prune ou reinício Docker. Não há APT novo nesta revisão; as camadas elegíveis de base/APT/colcon podem continuar em cache. Os scripts/configurações posteriores mudam. O smoke valida também os dois perfis completos e dependências estáticas do visor; não demonstra GUI funcional. Sem `source` de ROS nem instalação de pacotes no host.

**Coleta de perfis, sem movimento:** após o build validado, o bloco seguinte inicia e encerra dois containers diagnósticos independentes, sempre sem goal/Twist. Não é comparação A/B de desempenho nem ensaio a 20/25 km/h; verifica os valores realmente aplicados. Cada tentativa tem saída própria. Pode ser executado antes ou depois da observação visual, sem mudar o checkout.

```bash
(
  set -euo pipefail
  cd "$task_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
    --repo "$task_root/gps_truck_nav" \
    --build-dir "$review3_root/build" \
    --output-dir "$review3_root/probe-20kmh" \
    --navigation-profile 20kmh
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
    --repo "$task_root/gps_truck_nav" \
    --build-dir "$review3_root/build" \
    --output-dir "$review3_root/probe-25kmh" \
    --navigation-profile 25kmh
)
```

Ler `navigation_profile.json`, `speed_chain.json`, `plugin_parameters.json` e `readiness.json` em cada `runtime_probe/`. MPPI e smoother devem mostrar o mesmo teto prospectivo de 5,555555… ou 6,944444… m/s, yaw ±0,4 e ré −2,25. Valores efetivos não equivalem à velocidade do caminhão. O raio 11,6, limites físicos, `scale_velocities=false` e acelerações históricas permanecem explicitamente pendentes. Publishers do behavior_server que contornem o smoother continuam documentados; nenhuma interface de comando foi corrigida por simples redução de vmax.

**Abrir a observação visual:** após o probe canônico acima concluir OBSERVATIONS_COLLECTED, no mesmo terminal:

```bash
(
  set -euo pipefail
  cd "$task_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py start \
    --repo "$task_root/gps_truck_nav" \
    --build-dir "$review3_root/build" \
    --probe-dir "$review3_root/probe" \
    --output-dir "$review3_root/visual" \
    --duration 900
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py serve \
    --session-dir "$review3_root/visual" \
    --port 8765
)
```

Abrir no navegador do ipt-sim-01 a URL local impressa pelo `serve`. Gazebo Classic e RViz rodam dentro do container; o navegador recebe somente imagens da tela. Não há canal de mouse/teclado para mover modelos, resetar física ou publicar goals. A câmera inicial é predefinida; pan/zoom interativo não está habilitado. RViz mostra as camadas e os tópicos configurados; plano/trajetória vazios são esperados sem planejamento. O modo tem duração limitada, mantém a rede do container `none` e não monta display/ROS do host.

`Ctrl-C` no `serve` encerra a sessão própria. Para consultar ou parar explicitamente, usar outro terminal, substituindo somente o caminho final pelo valor de `review3_root` impresso acima:

```bash
read -r -p 'Caminho completo da pasta third-review desta tentativa: ' review3_root
task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
cd "$task_root/gps_truck_nav"
/usr/bin/python3 -I tools/auto_tuning/visual_observer.py status \
  --session-dir "$review3_root/visual"
/usr/bin/python3 -I tools/auto_tuning/visual_observer.py stop \
  --session-dir "$review3_root/visual"
```

Não usar kill/stop globais. O comando stop verifica ID e labels e preserva os logs. Para detalhes de falhas de GUI, timeout, captura opcional e limitações, usar `visual_observation_runbook.md`. A sessão GUI não prova parada de atuação; o encerramento de um container de simulação e a parada medida de um caminhão são critérios diferentes.

**Empacotar o retorno ao revisor**, após encerrar a visualização (se iniciada):

```bash
/usr/bin/python3 -I - "$review3_root" <<'PY'
from pathlib import Path
import hashlib
import json
import sys
import zipfile
root = Path(sys.argv[1]).resolve(strict=True)
archive = root / 'nav2_phase_a_third_review_evidence.zip'
allowed = {'.json', '.jsonl', '.log', '.txt', '.tsv', '.yaml', '.rviz', '.world', '.png'}
with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as output:
    for name in ('checkout_commit.txt', 'git_fetch.log'):
        path = root / name
        if path.is_file() and not path.is_symlink():
            output.write(path, path.relative_to(root))
    for name in ('build', 'probe', 'probe-20kmh', 'probe-25kmh', 'visual'):
        directory = root / name
        if not directory.is_dir() or directory.is_symlink():
            continue
        for path in sorted(directory.rglob('*')):
            if (path.is_file() and not path.is_symlink()
                    and path.resolve().is_relative_to(directory.resolve())
                    and (path.suffix in allowed or path.name == 'container.id')):
                output.write(path, path.relative_to(root))
digest = hashlib.sha256(archive.read_bytes()).hexdigest()
with (root / 'archive_identity.json').open('x') as stream:
    json.dump({'archive': archive.name, 'sha256': digest}, stream, indent=2)
    stream.write('\n')
print(archive)
print('SHA-256:', digest)
PY
```

O ZIP não inclui meshes, imagens Docker, framebuffer XWD nem dados históricos. Imagens PNG só existirão se a captura opcional tiver sido solicitada. Não apagar uma tentativa para repetir: criar outro `third-review.*`. No diagnóstico schema 2, `collection_status=COMPLETE` pode incluir ausência opcional comprovada, enquanto `readback_collection_status=PARTIAL` preserva que nem todo nome recebeu valor. `required_collection_status` deve continuar COMPLETE; UNDECLARED/UNSUPPORTED_VERSION nunca tornam um knob elegível ao tuning.

Retornar o ZIP ao revisor. O próximo movimento depende de revisão da atuação, geometria, autoridade, parada, percepção/localização e autorização posterior explícita. **Nenhum comando desta seção envia objetivo, Twist, missão ou inicia Optuna/Virtual Lanes.**

### 2.5. Candidato Classic coerente e observação visual

Ler `plant_interface_decision.md` e `prospective_plant_validation.md`. O candidato
mantém geometria/datum/sensores, acrescenta a referência planar `rear_axle_nav`,
translada o footprint sem mudar o corpo físico, usa raio 13.10 m derivado dos
limites existentes e alinha os tetos MPPI/smoother em 20/25 km/h. O tópico final
do plugin fica isolado em `/actuation/steering_command`, sem publisher. O
adaptador shadow recebe mensagens e grava cálculos, sem publicar comandos.
Isso prepara a validação da atuação; não concede autorização para movimento.

**No ipt-sim-01**, executar o bloco completo em Bash. Ele preserva o checkout
anterior e cria outro worktree a partir da PR. O SHA exato desta entrega consta
na resposta; para fixá-lo, pode substituir `FETCH_HEAD` no `worktree add` por
esse SHA. O novo diretório recebe todos os logs, sem remover imagens/containers
ou alterar Docker/Humble/CARLA/Autoware. Nenhum `source` ROS no host.

```bash
task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
plant_root=$(mktemp -d "$task_root/plant-review.XXXXXXXX")
printf 'Checkout e evidências desta tentativa: %s\n' "$plant_root"
(
  set -euo pipefail
  test -n "$plant_root"
  test -d "$plant_root"
  git -C "$task_root/gps_truck_nav" fetch origin engineering/2026-10-preflight-gates \
    2>&1 | tee "$plant_root/git_fetch.log"
  GIT_LFS_SKIP_SMUDGE=1 git -C "$task_root/gps_truck_nav" worktree add --detach \
    "$plant_root/gps_truck_nav" FETCH_HEAD
  cd "$plant_root/gps_truck_nav"
  git lfs checkout 2>&1 | tee "$plant_root/lfs_checkout.log"
  git lfs fsck 2>&1 | tee "$plant_root/lfs_fsck.log"
  test -z "$(git status --porcelain --untracked-files=all)"
  test -f config/auto_tuning/plant_candidate/manifest.json
  git rev-parse HEAD > "$plant_root/checkout_commit.txt"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$plant_root/gps_truck_nav" --output-dir "$plant_root/build"
  /usr/bin/python3 -I - "$plant_root/build/status.json" <<'PY'
import json
import sys
if json.load(open(sys.argv[1])).get('status') != 'IMAGE_VALIDATED':
    raise SystemExit('Imagem não validada. Preservar os logs; não iniciar o probe.')
PY
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
    --repo "$plant_root/gps_truck_nav" --build-dir "$plant_root/build" \
    --output-dir "$plant_root/probe" --navigation-profile candidate20kmh
)
```

`git lfs checkout` reaproveita os objetos locais. Se `fsck` indicar objeto
ausente, interromper esse bloco e executar `git -C "$plant_root/gps_truck_nav"
lfs pull` para hidratar apenas o checkout dedicado, preservando o log numa nova
tentativa. Não limpar evidências para contornar erro de fonte. APT/base continuam
elegíveis ao cache; os dois launches mudaram, então colcon/reinstalação do pacote
podem ser refeitos. Não usar `--no-cache`, prune ou reiniciar Docker.

O novo smoke verifica também o bundle reproduzível na disposição **instalada**,
o argumento de URDF e a CLI shadow. A saída continua separada em:

| Evidência | O que verificar |
|---|---|
| `build/status.json` | IMAGE_VALIDATED, imagem por ID e source manifest |
| `probe/status.json` | OBSERVATIONS_COLLECTED, estado diagnóstico separado, cleanup próprio |
| `probe/runtime_probe/readiness.json` | lifecycle/TF/sensores/parâmetros anteriores e checks do candidato |
| `probe/runtime_probe/navigation_profile.json` | candidate20kmh e hashes da seleção |
| `probe/runtime_probe/plugin_parameters.json` | readback requerido completo, valores de Smac/MPPI realmente retornados |
| `probe/runtime_probe/speed_chain.json` | tetos efetivos e publishers que continuam fora do smoother |
| `probe/runtime_probe/plant_candidate.json` | seleção e contrato da planta prospectiva |
| `probe/actuation_shadow/shadow_summary.json` | zero publishers criados, ausência de entrada não nula/inválida; NOT_EXERCISED esperado sem goals |
| `probe/actuation_shadow/shadow_commands.jsonl` | entradas realmente recebidas, se existirem; nenhum estímulo é gerado |

Os checks `candidate_actuation_topic_isolated` e
`candidate_rear_axle_reference` precisam demonstrar o subscriber privado, zero
publishers e frame fixo/autoridade correta. Conferir os nomes exatos registrados
no JSON se a ferramenta reportar erro. A ausência de comando não prova conversão
dinâmica. `SHADOW_COMPLETE` significa coleta concluída; mission_ready permanece
false e diagnóstico bloqueado para atuação. Qualquer entrada não nula/NaN/Inf
observada no shadow aborta a tentativa, preserva os dados e encerra seu container.

**Abrir Gazebo Classic/RViz em observação**, após OBSERVATIONS_COLLECTED desta
mesma imagem/seleção, sem atualizar o worktree:

```bash
(
  set -euo pipefail
  cd "$plant_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py start \
    --repo "$plant_root/gps_truck_nav" --build-dir "$plant_root/build" \
    --probe-dir "$plant_root/probe" --output-dir "$plant_root/visual" --duration 900
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py serve \
    --session-dir "$plant_root/visual" --port 8765
)
```

Abrir **no navegador do ipt-sim-01** a URL localhost impressa. São os pixels de
Gazebo/RViz reais no container; o visor não transmite mouse/teclado. Observar
Arocs, rodas, sensores, TF e footprint/costmaps. Plano/trajetória vazios são
esperados nesta etapa sem goals. Pan/zoom/objetivo interativos ficam para a etapa
supervisionada autorizada. A seleção candidate20kmh é herdada do probe; não se
abre silenciosamente a planta canônica. A sessão encerra em até 900 s e conserva
logs. Não tratar interface visível como validação de parada ou percepção.

`Ctrl-C` no serve encerra a sessão própria. Em outro terminal, estes comandos
consultam e encerram somente a sessão cujo ID/labels foram registrados:

```bash
read -r -p 'Caminho completo da pasta plant-review desta tentativa: ' plant_root
cd "$plant_root/gps_truck_nav"
/usr/bin/python3 -I tools/auto_tuning/visual_observer.py status --session-dir "$plant_root/visual"
/usr/bin/python3 -I tools/auto_tuning/visual_observer.py stop --session-dir "$plant_root/visual"
```

**Opcional, somente readback do teto posterior de 25 km/h**, nunca um ensaio
nessa velocidade:

```bash
cd "$plant_root/gps_truck_nav"
/usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
  --repo "$plant_root/gps_truck_nav" --build-dir "$plant_root/build" \
  --output-dir "$plant_root/probe-25kmh" --navigation-profile candidate25kmh
```

Depois de encerrar a visualização, empacotar apenas as evidências novas:

```bash
/usr/bin/python3 -I - "$plant_root" <<'PY'
from pathlib import Path
import hashlib
import json
import sys
import zipfile
root = Path(sys.argv[1]).resolve(strict=True)
archive = root / 'nav2_prospective_plant_evidence.zip'
allowed = {'.json', '.jsonl', '.log', '.txt', '.tsv', '.yaml', '.rviz', '.world', '.png'}
with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as out:
    for name in ('checkout_commit.txt', 'git_fetch.log', 'lfs_checkout.log', 'lfs_fsck.log'):
        path = root / name
        if path.is_file() and not path.is_symlink():
            out.write(path, path.relative_to(root))
    for name in ('build', 'probe', 'probe-25kmh', 'visual'):
        directory = root / name
        if not directory.is_dir() or directory.is_symlink():
            continue
        for path in sorted(directory.rglob('*')):
            if (path.is_file() and not path.is_symlink()
                    and path.resolve().is_relative_to(directory.resolve())
                    and (path.suffix in allowed or path.name == 'container.id')):
                out.write(path, path.relative_to(root))
identity = {'archive': archive.name, 'sha256': hashlib.sha256(archive.read_bytes()).hexdigest()}
with (root / 'archive_identity.json').open('x') as stream:
    json.dump(identity, stream, indent=2)
    stream.write('\n')
print(archive)
print('SHA-256:', identity['sha256'])
PY
```

O retorno útil é esse ZIP e as observações visuais. Não repetir indefinidamente
probes para eliminar warnings: avaliar apenas falhas que afetem a cadeia de
comando, geometria, localização/percepção do percurso ou a evidência necessária.
Após revisão, a próxima etapa será autorização de um ensaio delimitado de
conversão/dinâmica/parada; primeiro objetivo de navegação vem depois desse aceite.
O protocolo em `prospective_plant_validation.md` define os critérios e registros.
O caminho final de publicação com supervisão independente ainda precisa ser
conectado e ensaiado nessa etapa, não está ativo aqui.

O framework compara parâmetros de referência e otimizados **com a mesma planta
validada fixa**. Histórico fica rastreável; não é necessário benchmark
histórico/corrigido. Benchmark entre pares planejador–controlador é posterior e
condicionado ao tempo. Nenhum comando desta seção publica Twist, goal, missão,
inicia Optuna, fine-tuning ou Virtual Lanes.

## 3. Depois de IMAGE_VALIDATED: bringup headless diagnóstico

**Diretório de trabalho:** o mesmo checkout novo. **Pré-requisitos:** `task_root` mantido no terminal, `build/status.json` aprovado pelo smoke check, imagem existente no daemon local e checkout intacto. Em outro terminal, recupere o caminho impresso na seção 2 e atribua-o a `task_root` antes de continuar; não escolha arbitrariamente um build antigo.

```bash
cd "$task_root/gps_truck_nav"
/usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
  --repo "$task_root/gps_truck_nav" \
  --build-dir "$task_root/build" \
  --output-dir "$task_root/probe"
```

Esse estágio confere a imagem e a fonte, cria um container novo com limites e rede isolada, inicia Xvfb/Gazebo/localização/Nav2 sem RViz/Mapviz/gzclient, usa o overlay prospectivo de localização e `publish_legacy_cloud_tf:=false`, aguarda somente um aquecimento limitado e coleta probes. Ele **não espera prontidão para enviar goals**, não inicia recorder/mission sender/optimizer, não injeta comandos Twist e não altera raios/footprint/limites físicos.

A coleta tem timeouts e pode durar vários minutos. `tf2_echo` costuma atingir seu timeout após coletar amostras; é necessário ler o log, não interpretar esse exit code isoladamente como erro ou aprovação. O helper registra stdout/stderr do bringup, logs do Xvfb, inspect anterior/posterior e confirma parada `Running=false`, `Pid=0` do ID que criou. Containers próprios ficam parados para inspeção; não são removidos automaticamente.

`OBSERVATIONS_COLLECTED` significa somente coleta e encerramento concluídos. O resumo atual também informa `diagnostic_status`, `readiness_collection_status` e `mission_ready=false`; interpretar conforme seção 2.3. Sensores, TF, lifecycle, parâmetros e estado dos processos precisam ser revisados; várias consultas podem ter falhado. Um processo de launch vivo não basta para provar que gzserver e componentes Nav2 continuaram vivos. Nenhum status desta etapa aprova Gate 0/1.

Se houver falta de RAM, erro de software rendering, OOM ou falha de imagem, preservar logs. Não alterar drivers NVIDIA, habilitar `--privileged`, usar rede do host ou reiniciar Docker. Uma eventual alternativa de renderização deve ser uma alteração explícita e revisada do ambiente.

## 4. Evidências e recuperação

O helper salva `status.json`, `source_manifest.json`, `commands.jsonl` e logs por comando em cada saída. Build também preserva `image_id.txt`, `image_inspect.json` e `installed-packages.tsv`. Probe inclui `runtime_probe/`, `bringup.log`, `xvfb.log` quando gerado, `container.id`, `container_before_stop.json` e `container_after_stop.json`. Os IDs são específicos desta execução. Nunca usar `pkill`, `docker stop $(docker ps ...)`, `docker system prune`, remoção de volumes compartilhados ou reset global.

Se o terminal/host for encerrado à força ou ocorrer outra interrupção durante o cleanup, ele pode não concluir. Recuperar o diretório correto e usar o procedimento explícito de emergência descrito abaixo somente para o ID salvo. São conferidos o label do projeto e o identificador da tentativa:

```bash
(
  set -euo pipefail
  stage_dir="$task_root/probe"
  docker_local=(env -u DOCKER_HOST -u DOCKER_CONTEXT -u DOCKER_TLS \
    -u DOCKER_TLS_VERIFY -u DOCKER_CERT_PATH -u DOCKER_API_VERSION \
    /usr/bin/docker --host unix:///var/run/docker.sock)
  container_id=$(cat "$stage_dir/container.id")
  case "$container_id" in
    ''|*[!0-9a-f]*) exit 1 ;;
  esac
  test "${#container_id}" -eq 64
  run_id=$(/usr/bin/python3 -I -c \
    'import json,sys; print(json.load(open(sys.argv[1]))["run_id"])' \
    "$stage_dir/status.json")
  case "$run_id" in
    ''|*[!0-9a-f]*) exit 1 ;;
  esac
  test "${#run_id}" -eq 32
  owner=$("${docker_local[@]}" inspect \
    --format '{{index .Config.Labels "project"}}' "$container_id")
  test "$owner" = nav2-phase-a
  actual_run=$("${docker_local[@]}" inspect \
    --format '{{index .Config.Labels "nav2.phase_a.run_id"}}' "$container_id")
  test "$actual_run" = "$run_id"
  "${docker_local[@]}" stop --time 20 "$container_id"
  "${docker_local[@]}" inspect \
    --format '{{.State.Running}} {{.State.Pid}}' "$container_id"
)
```

Para falha no validador da imagem, trocar apenas `stage_dir="$task_root/probe"` por `stage_dir="$task_root/build"` nesse bloco completo. A rotina normal já realiza a parada, sem intervenção. O comando de emergência não transforma a tentativa interrompida em sucesso; confirme a saída `false 0` e preserve a tentativa.

Para enviar as evidências do build/probe sem incluir imagem, fontes ou outros ambientes, executar no mesmo terminal:

```bash
/usr/bin/python3 -I - "$task_root" <<'PY'
from pathlib import Path
import sys
import zipfile
root = Path(sys.argv[1]).resolve(strict=True)
archive = root / 'phase_a_evidence.zip'
allowed = {'.json', '.jsonl', '.log', '.txt', '.tsv', '.yaml'}
with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as output:
    for name in ('checkout_commit.txt', 'lfs_pull.log', 'lfs_fsck.log'):
        path = root / name
        if path.is_file() and not path.is_symlink():
            output.write(path, path.relative_to(root))
    for name in ('build', 'probe'):
        directory = root / name
        if not directory.is_dir() or directory.is_symlink():
            continue
        for path in sorted(directory.rglob('*')):
            if (path.is_file() and not path.is_symlink()
                    and path.resolve().is_relative_to(directory.resolve())
                    and (path.suffix in allowed or path.name == 'container.id')):
                output.write(path, path.relative_to(root))
print(archive)
PY
```

O ZIP usa criação exclusiva: não sobrescreve um pacote anterior. Logs podem conter caminhos e detalhes de configuração desse novo experimento; revise antes de encaminhá-los ao revisor. Não incluir os 212 bags históricos nem diretórios CARLA/Autoware.

Para repetir uma etapa com falha, criar outro diretório de saída, preservando o anterior. Corrigir o motivo da falha antes de repetir; nunca apagar logs para fazer o mesmo caminho ser aceito. Nova revisão de código exige novo build; não editar o manifesto para reutilizar uma imagem incompatível.

## 5. Congelamento e bloqueios antes de missão

O digest da base e os hashes de fonte são registrados; APT ainda é resolvido durante o build. Antes de iniciar comparação experimental, preservar a imagem final e planejar lock de dependências. Para exportar a imagem validada, com espaço livre suficiente e destino ainda inexistente:

```bash
(
  set -euo pipefail
  image_id=$(cat "$task_root/build/image_id.txt")
  test ! -e "$task_root/runtime-image.tar"
  env -u DOCKER_HOST -u DOCKER_CONTEXT -u DOCKER_TLS \
    -u DOCKER_TLS_VERIFY -u DOCKER_CERT_PATH -u DOCKER_API_VERSION \
    /usr/bin/docker --host unix:///var/run/docker.sock image save \
    --output "$task_root/runtime-image.tar" "$image_id"
  sha256sum "$task_root/runtime-image.tar" > "$task_root/runtime-image.tar.sha256"
)
```

A imagem exportada permite replay daquele runtime; não prova rebuild independente bit a bit. Não enviar esse TAR junto dos logs do diagnóstico.

Costmaps e BTs foram confirmadas no commit 978ab23 e continuam protegidas por regressões. Permanecem abertos: interface yaw-rate/esterçamento e limites físicos; validação runtime da variante TF/frames; precisão e tempo interno de localização, covariâncias e lever arms; produtor externo de obstáculos; consumo efetivo dos knobs de otimização; associação da missão pública às referências; velocidade efetiva e equivalente geométrico do evaluator. A próxima decisão depende da revisão das novas evidências locais. **PR #2 permanece draft, sem merge; missão, Gates 0/1, otimização e campanha experimental não foram autorizados por um smoke check.**

Fontes de operação consultadas: [rede none](https://docs.docker.com/engine/network/drivers/none/), [limites de runtime](https://docs.docker.com/engine/containers/resource_constraints/), [builders](https://docs.docker.com/build/builders/), [storage containerd](https://docs.docker.com/engine/storage/containerd/), [Mesa software](https://docs.mesa3d.org/envvars.html), [xvfb-run Jammy](https://manpages.ubuntu.com/manpages/jammy/man1/xvfb-run.1.html).
