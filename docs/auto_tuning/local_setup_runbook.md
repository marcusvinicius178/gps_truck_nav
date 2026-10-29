# Preparação local — Fase A, 29/10/2026

**Estado: PROPOSED BUT NOT TESTED no ROS/Docker.** Os testes Python são separados da validação ROS. Este roteiro prepara um ambiente prospectivo; não reproduz automaticamente o runtime privado nem aprova Gates 0/1. Não instalar ROS no host nesta etapa. Primeiro executar apenas o diagnóstico da seção 1 e retornar o JSON.

## 1. Diagnóstico imediato, sem instalar nada

Pré-requisitos: Linux e Python 3. Se o arquivo `host_preflight.py` foi baixado para `~/Downloads`, executar no terminal do computador que rodará a simulação:

```bash
cd "$HOME/Downloads"
diagnostic_dir=$(mktemp -d "$HOME/nav2-host-diagnostic.XXXXXXXX")
python3 host_preflight.py --probe-docker --output "$diagnostic_dir/host_preflight.json"
printf 'Enviar este arquivo: %s\n' "$diagnostic_dir/host_preflight.json"
```

Se o checkout de engenharia já existe, executar na raiz dele, acrescentando `--repo .`. Ausência de ROS/Gazebo no host é aceitável com Docker. O script não instala pacotes nem inicia containers; o exit code 0 significa apenas que coletou o diagnóstico. Não há acesso remoto ao computador por este Work.

## 2. Preparar Ubuntu e Docker, somente após conhecer o diagnóstico

Alvo inicial: host Linux x86_64; container Ubuntu 22.04/Iron. Host Ubuntu 22.04 ou 24.04 é uma opção direta. Não substituir a distribuição ROS. Como orçamento de planejamento, reservar 16 GiB RAM, 4+ cores e 50 GiB livres; 32 GiB/8 cores/100 GiB oferecem folga. **Não são mínimos medidos**: o piloto precisa medir RAM, CPU, disco, real-time factor e prazos do controller. O disco adicional depende de MCAP, imagens e duração.

Em terminal no host, diretório `$HOME`; pré-requisitos: Ubuntu suportado, internet e sudo. Instalar ferramentas de preparação:

```bash
cd "$HOME"
sudo apt-get update
sudo apt-get install -y git git-lfs python3 python3-yaml ca-certificates curl
```

Se Docker já funciona, não reinstalar. Para uma **instalação nova**, sem Docker/containerd/runc existentes, os comandos abaixo configuram o repositório oficial. Se já houver pacotes conflitantes, resolver sua coexistência antes; este roteiro não remove serviços existentes automaticamente.

```bash
cd "$HOME"
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
ubuntu_codename=$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
host_arch=$(dpkg --print-architecture)
sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: ${ubuntu_codename}
Components: stable
Architectures: ${host_arch}
Signed-By: /etc/apt/keyrings/docker.asc
EOF
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl start docker
sudo docker version
sudo docker compose version
```

Fonte consultada em 29/10/2026: <https://docs.docker.com/engine/install/ubuntu/>. Nos próximos comandos, usar `sudo docker` se o usuário ainda não possui acesso ao daemon local. Não é necessário conceder acesso ao grupo Docker para executar este roteiro. Os comandos abaixo usam explicitamente o socket local `/var/run/docker.sock`; Docker rootless precisa de seu socket real, conforme diagnóstico.

## 3. Checkout novo e modelos LFS

Diretório inicial `$HOME`; pré-requisitos: Git/LFS/PyYAML instalados e diretório de destino ainda inexistente. Não executar estes comandos sobre um workspace antigo com alterações.

```bash
cd "$HOME"
set -euo pipefail
mkdir nav2_autotuning_20261029
cd nav2_autotuning_20261029
GIT_LFS_SKIP_SMUDGE=1 git clone --branch engineering/2026-10-preflight-gates \
  https://github.com/marcusvinicius178/gps_truck_nav.git
git clone https://github.com/marcusvinicius178/nav2_paper_scripts.git
git -C nav2_paper_scripts checkout --detach 36ba76823cc7a124540412cf3ce09c73e6cd1d05
cd gps_truck_nav
git lfs install --local
git lfs pull
git lfs fsck
python3 tools/audit_repository.py
python3 -m unittest discover -s tests -v
```

As sete meshes LFS representam 277.750.112 bytes de payload único. O clone Git sem `lfs pull` não basta. O build deve parar se o audit identificar ponteiros. Os arquivos completos desta entrega estão no branch indicado; nenhuma inserção manual em arquivos é necessária.

## 4. Build de descoberta e congelamento

Na raiz `~/nav2_autotuning_20261029/gps_truck_nav`; pré-requisitos: audit aprovado, Docker local acessível, internet, LFS hidratado. Executar o bloco inteiro no mesmo terminal. Ele não inicia uma missão.

```bash
cd "$HOME/nav2_autotuning_20261029/gps_truck_nav"
set -euo pipefail
session_dir=$(mktemp -d "$HOME/nav2-preflight.XXXXXXXX")
git rev-parse HEAD > "$session_dir/integration_commit.txt"
git -C ../nav2_paper_scripts rev-parse HEAD > "$session_dir/analysis_commit.txt"
git status --porcelain > "$session_dir/source_status.txt"
test ! -s "$session_dir/source_status.txt"
sudo docker --host unix:///var/run/docker.sock pull osrf/ros:iron-desktop-full \
  2>&1 | tee "$session_dir/base_pull.log"
base_digest=$(sudo docker --host unix:///var/run/docker.sock image inspect \
  osrf/ros:iron-desktop-full --format '{{index .RepoDigests 0}}')
case "$base_digest" in
  osrf/ros@sha256:*) ;;
  *) printf 'RepoDigest inesperado; interromper e revisar: %s\n' "$base_digest" >&2; exit 1 ;;
esac
printf '%s\n' "$base_digest" > "$session_dir/base_digest.txt"
sudo docker --host unix:///var/run/docker.sock build --progress=plain \
  --build-arg ROS_BASE="$base_digest" \
  --iidfile "$session_dir/image_id.txt" \
  -f docker/auto_tuning/Dockerfile . 2>&1 | tee "$session_dir/build.log"
image_id=$(cat "$session_dir/image_id.txt")
sudo docker --host unix:///var/run/docker.sock run --rm --network none \
  --entrypoint cat "$image_id" /ws/installed-packages.tsv > "$session_dir/installed-packages.tsv"
sudo docker --host unix:///var/run/docker.sock image inspect "$image_id" > "$session_dir/image_inspect.json"
python3 tools/auto_tuning/host_preflight.py --repo . --output "$session_dir/host_preflight.json"
python3 tools/auto_tuning/prepare_candidates.py \
  --source src/gps_truck_nav/truck_bringup/params/truck_nav2_params.yaml \
  --output-dir "$session_dir/candidates"
printf 'Evidências e candidatos: %s\n' "$session_dir"
```

O Dockerfile novo compila somente `truck_bringup` contra Nav2 binário Iron instalado e inclui explicitamente MCAP. Não executa `vcs import`, não compila Virtual Lanes e não mistura o SHA Nav2 opcional com binários. Seu `.dockerignore` próprio inclui as fontes, pois o `.dockerignore` original as exclui. `BUILD_TESTING=OFF` aplica-se ao build de descoberta; validação do plugin em runtime continua obrigatória.

**O digest da base não congela o APT.** A primeira imagem resolve pacotes disponíveis naquele momento. Registrar a lista instalada e conservar a imagem resultante é necessário para repetir o mesmo runtime. Antes de comparar candidatos, preservar a imagem no mesmo terminal (requer espaço adicional):

```bash
sudo docker --host unix:///var/run/docker.sock image save "$image_id" | gzip > "$session_dir/runtime-image.tar.gz"
sha256sum "$session_dir/runtime-image.tar.gz" > "$session_dir/runtime-image.tar.gz.sha256"
```

Se `pull` ou APT falhar, enviar log; não trocar Iron, eliminar assinatura APT, remover dependências ou declarar indisponibilidade total com base em uma falha. Um lock de pacotes/repositório de artefatos para rebuild independente ainda será necessário. A imagem local salva dá replay do runtime, não prova rebuild bit a bit da receita.

## 5. Bringup diagnóstico isolado, sem missão

Pré-requisitos: build concluído; manter `session_dir` e `image_id` da seção 4. O container é novo, sem rede externa, sem volumes de build antigos, sem dados históricos e sem X11. O nome único evita interferir em outro experimento.

```bash
cd "$HOME/nav2_autotuning_20261029/gps_truck_nav"
(
  set -Eeuo pipefail
  container_name="nav2-preflight-$(basename "$session_dir")"
  cid_file="$session_dir/container.id"
  test ! -e "$cid_file"
  cleanup() {
    result_code=$?
    trap - EXIT INT TERM
    if [[ -s "$cid_file" ]]; then
      container_id=$(cat "$cid_file")
      sudo docker --host unix:///var/run/docker.sock logs "$container_id" > "$session_dir/bringup.log" 2>&1 || true
      sudo docker --host unix:///var/run/docker.sock inspect "$container_id" > "$session_dir/container_before_stop.json" || true
      sudo docker --host unix:///var/run/docker.sock stop --time 20 "$container_id" || result_code=1
      sudo docker --host unix:///var/run/docker.sock inspect "$container_id" > "$session_dir/container_after_stop.json" || result_code=1
      stopped_state=$(sudo docker --host unix:///var/run/docker.sock inspect \
        --format '{{.State.Running}} {{.State.Pid}}' "$container_id") || result_code=1
      printf 'Estado após parada: %s\n' "$stopped_state"
      [[ "$stopped_state" == 'false 0' ]] || result_code=1
    fi
    exit "$result_code"
  }
  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  sudo docker --host unix:///var/run/docker.sock run -d --init \
    --cidfile "$cid_file" --name "$container_name" --network none --shm-size=1g \
    -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=71 \
    --mount "type=bind,src=$session_dir,dst=/evidence" \
    "$image_id" ros2 launch truck_bringup gps_waypoint_follower.launch.py \
    use_gzclient:=false use_rviz:=false use_mapviz:=false use_gpu:=false use_xcb:=false \
    publish_legacy_cloud_tf:=false \
    localization_params_file:=/opt/auto_tuning/config/dual_ekf_navsat_gate0.yaml
  container_id=$(cat "$cid_file")
  sudo docker --host unix:///var/run/docker.sock exec "$container_id" /entrypoint.sh \
    bash /opt/auto_tuning/tools/collect_runtime_evidence.sh /evidence/runtime_probe
)
```

O coletor faz probes sequenciais com timeout de 15 s; pode levar vários minutos. `tf2_echo` normalmente termina por timeout após coletar amostras. Logs podem ser escritos como root no diretório montado; leitura permanece possível. A coleta deve ser revisada: clock avançando, covariâncias/frames, única autoridade TF, pose inicial, GPS convertido, lifecycle e parâmetros efetivos. `false 0` ao final é evidência de parada do container, mas **não demonstra sozinho reset ROS/Gazebo entre duas missões**. Manter o container parado até revisar os logs; não usar `docker system prune` nem comandos globais de kill.

O trap tenta finalizar somente o ID criado mesmo quando o probe falha. Se o host/terminal for encerrado à força, recuperar o diretório de evidências da seção 4 e parar apenas o ID salvo:

```bash
container_id=$(cat "$session_dir/container.id")
sudo docker --host unix:///var/run/docker.sock logs "$container_id" > "$session_dir/bringup_failure.log" 2>&1
sudo docker --host unix:///var/run/docker.sock stop --time 20 "$container_id"
```

O overlay é uma hipótese de reparo: Gazebo publica `odom → base_footprint`; EKF global publica `map → odom`; EKF local só fornece `/odometry/local`; URDF fornece `base_link → cloud`. Validar no mundo real da simulação, inclusive covariance, timestamps e consistência entre odometria local e TF Gazebo. Não substituir o baseline congelado sem registrar o overlay.

## 6. Próxima etapa após o bringup

Enviar `host_preflight.json`, `build.log`, `installed-packages.tsv`, `bringup.log` e `runtime_probe/`, além dos dois JSONs de estado do container. Não enviar os 212 bags históricos. Antes de qualquer goal, verificar a interface Ackermann instalada: no plugin Gazebo upstream 3.7.0, `Twist.angular.z` representa esterçamento, enquanto MPPI fornece yaw rate. O caminho público não mostra conversor. Resolver essa interface com geometria validada, os frames de antena/IMU e o BT; depois fechar associação missão/referência, wheelbase equivalente do avaliador e limites de prontidão.

As etapas exatas de Gate 0/1 e as pendências de implementação estão em `gate0_gate1_plan.md`. **Não existe nesta entrega um runner fim a fim aprovado**; há diagnóstico, candidatos, overlay e adaptador de métricas. A operação local acima é bringup diagnóstico, não Gate 0 nem Gate 1.
