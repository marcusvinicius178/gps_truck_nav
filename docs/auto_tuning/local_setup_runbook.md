# Preparação local — ipt-sim-01, continuação da Fase A

**HOST PREFLIGHT APROVADO pelo revisor. Gates 0 e 1: NOT YET VERIFIED.** O próximo passo é construir uma imagem isolada Iron/Classic e verificar seu bringup sem enviar missão. Esta revisão substitui as instruções anteriores de instalação do host.

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

`OBSERVATIONS_COLLECTED` significa somente coleta e encerramento concluídos. Sensores, TF, lifecycle, parâmetros e estado dos processos precisam ser revisados; várias consultas podem ter falhado. Um processo de launch vivo não basta para provar que gzserver e componentes Nav2 continuaram vivos. Nenhum status desta etapa aprova Gate 0/1.

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

Mesmo se build e bringup funcionarem, permanecem abertos: interface yaw-rate do MPPI versus esterçamento do Ackermann upstream; TF/localização e frames/lever arms de GPS/IMU; BT/Spin e costmaps; associação da missão pública às referências; velocidade efetiva e equivalente geométrico do evaluator. O perfil de localização é proposto, não validado. A próxima decisão depende da inspeção das evidências locais. **PR #2 permanece draft, sem merge; missão, Gates 0/1, otimização e campanha experimental não foram autorizados por um smoke check.**

Fontes de operação consultadas: [rede none](https://docs.docker.com/engine/network/drivers/none/), [limites de runtime](https://docs.docker.com/engine/containers/resource_constraints/), [builders](https://docs.docker.com/build/builders/), [storage containerd](https://docs.docker.com/engine/storage/containerd/), [Mesa software](https://docs.mesa3d.org/envvars.html), [xvfb-run Jammy](https://manpages.ubuntu.com/manpages/jammy/man1/xvfb-run.1.html).
