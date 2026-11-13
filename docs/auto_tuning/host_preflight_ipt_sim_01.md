# Host preflight — ipt-sim-01

**Decisão do revisor: HOST PREFLIGHT APROVADO. Gates 0 e 1: NOT YET VERIFIED.**

Fonte: `host_preflight.json` fornecido pelo proprietário, lido integralmente; coleta em 29/10/2026 às 19:00:40 UTC (16:00:40 em São Paulo). Este documento resume o diagnóstico, sem publicar o JSON pessoal completo. A execução ocorreu no host pelo proprietário; o agente não executou comandos remotamente nessa máquina.

| Item | Observado no diagnóstico |
|---|---|
| Sistema | Ubuntu 22.04.5 LTS, x86_64, kernel 6.8.0-138-generic |
| Python | 3.10.12 em `/usr/bin/python3` |
| CPU | 32 CPUs lógicas, afinidade 32; sem limite cgroup observado |
| RAM | 60,3 GiB total; 40,8 GiB disponíveis no instante da coleta |
| Disco | 366,6 GiB livres no filesystem de Downloads |
| Docker | Engine 29.8.2, Compose v5.5.1; daemon Linux local respondeu |
| Endpoint | Contexto default, `unix:///var/run/docker.sock`, sem overrides de host/contexto |
| Git / LFS | 2.34.1 / 3.0.2 |
| ROS no host | Humble instalado; deve permanecer intacto |
| Gazebo no host | `gzserver`/`gazebo` não encontrados no PATH; não requer instalação no host |
| Dispositivos | NVIDIA e DRM presentes; funcionamento de renderização não foi testado |

O espaço de Downloads não comprova espaço do DockerRootDir nem do image store containerd. O helper da próxima etapa registra DockerRootDir, driver/status de storage e espaço observável nos filesystems relevantes. O caminho padrão containerd é apenas uma observação candidata, não detecção garantida de um root customizado. Nenhuma medição de capacidade garante desempenho de simulação em tempo real.

## Próxima execução autorizada

Seguir `local_setup_runbook.md`: checkout novo, hidratação LFS, build Docker de ROS 2 Iron/Gazebo Classic 11, smoke checks da imagem e bringup diagnóstico headless sem missão. O host usa somente Git, Docker e Python stdlib existentes. Não instalar/atualizar Docker, ROS, drivers ou dependências Python no host; não reutilizar diretórios, redes ou volumes de CARLA/Autoware.

O runtime usa rede isolada e Xvfb/Mesa por software, sem compartilhar GPU. Isso é uma escolha inicial a validar, não resultado observado. O daemon Docker e seu cache/disco continuam compartilhados; build/probe consomem recursos do host. Não reiniciar/reconfigurar o daemon nem remover recursos de outros projetos.

## O que esta aprovação não demonstra

Pull/base/APT, build C++, bibliotecas em runtime, renderer, sensores, TF/localização e Nav2 ainda não foram executados neste host pelo fluxo novo. A interface yaw-rate/esterçamento Ackermann e os demais bloqueios da PR #2 permanecem impeditivos de missão. Não há aprovação de Gate 0, Gate 1, reset, otimização ou campanha. A PR #2 permanece em rascunho, sem merge.
