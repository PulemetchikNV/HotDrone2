#!/bin/bash

# Скрипт для тестирования кластерной настройки
# Использование: ./test_cluster.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLUSTER_HOSTS_FILE="${CLUSTER_HOSTFILE:-cluster_hosts}"

# Функция логирования
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Тестирование конфигурации дронов
test_drone_config() {
    log "=== Testing drone configuration ==="
    
    python3 -c "
import sys
sys.path.append('drone')
try:
    from const import DRONES_CONFIG, get_drone_config
    print('Available drones:')
    for drone_name, config in DRONES_CONFIG.items():
        print(f'  {drone_name}: {config}')
        
    print('\nTesting get_drone_config():')
    for drone_name in DRONES_CONFIG.keys():
        try:
            config = get_drone_config(drone_name)
            ip = config.get('raw_ip', 'NO_IP')
            print(f'  {drone_name} -> IP: {ip}')
        except Exception as e:
            print(f'  {drone_name} -> ERROR: {e}')
            
except ImportError as e:
    print(f'Import error: {e}')
    sys.exit(1)
except Exception as e:
    print(f'Configuration error: {e}')
    sys.exit(1)
"
    
    if [ $? -ne 0 ]; then
        log "ERROR: Drone configuration test failed"
        return 1
    fi
    
    log "Drone configuration test passed"
    return 0
}

# Тестирование cluster_manager
test_cluster_manager() {
    log "=== Testing cluster manager ==="
    
    python3 -c "
import sys
sys.path.append('drone')
try:
    from cluster_manager import ClusterManager
    import logging
    
    # Создаем простой логгер
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger('test')
    
    cluster_mgr = ClusterManager(logger)
    
    # Тестируем получение IP
    from const import DRONES_CONFIG
    test_drones = list(DRONES_CONFIG.keys())
    ips = cluster_mgr.get_alive_drone_ips(test_drones)
    print(f'IPs for drones {test_drones}: {ips}')
    
    # Тестируем чтение cluster_hosts
    current_hosts = cluster_mgr.read_current_cluster_hosts()
    print(f'Current cluster_hosts: {current_hosts}')
    
    print('Cluster manager test passed')
    
except ImportError as e:
    print(f'Import error: {e}')
    sys.exit(1)
except Exception as e:
    print(f'Cluster manager error: {e}')
    sys.exit(1)
"
    
    if [ $? -ne 0 ]; then
        log "ERROR: Cluster manager test failed"
        return 1
    fi
    
    log "Cluster manager test passed"
    return 0
}

# Тестирование MPI
test_mpi() {
    log "=== Testing MPI setup ==="
    
    if ! command -v mpirun &> /dev/null; then
        log "WARNING: mpirun not found - install OpenMPI for cluster support"
        return 1
    fi
    
    log "mpirun found: $(which mpirun)"
    
    # Проверяем версию MPI
    mpirun --version 2>/dev/null | head -n 1
    
    # Создаем тестовый cluster_hosts если его нет
    if [ ! -f "$CLUSTER_HOSTS_FILE" ]; then
        log "Creating test cluster_hosts file"
        echo "localhost" > "$CLUSTER_HOSTS_FILE"
    fi
    
    log "Cluster hosts file ($CLUSTER_HOSTS_FILE):"
    cat "$CLUSTER_HOSTS_FILE" | sed 's/^/  /'
    
    return 0
}

# Тестирование Stockfish
test_stockfish() {
    log "=== Testing Stockfish setup ==="
    
    python3 -c "
import sys
sys.path.append('drone')
try:
    from alg2_stockfish import _find_stockfish_binary_for_cluster, test_stockfish_integration
    
    # Ищем бинарник Stockfish
    binary = _find_stockfish_binary_for_cluster()
    if binary:
        print(f'Stockfish binary found: {binary}')
    else:
        print('Stockfish binary not found')
        
    # Тестируем интеграцию
    result = test_stockfish_integration()
    if result.get('status') == 'success':
        print(f'Stockfish integration test passed')
        print(f'  Test move: {result.get(\"best_move\")}')
        print(f'  Evaluation: {result.get(\"evaluation\")}')
    else:
        print(f'Stockfish integration test failed: {result.get(\"message\")}')
        
except ImportError as e:
    print(f'Import error: {e}')
    sys.exit(1)
except Exception as e:
    print(f'Stockfish test error: {e}')
    sys.exit(1)
"
    
    if [ $? -ne 0 ]; then
        log "ERROR: Stockfish test failed"
        return 1
    fi
    
    log "Stockfish test passed"
    return 0
}

# Демонстрация кластерного обновления
demo_cluster_update() {
    log "=== Demo: Cluster update simulation ==="
    
    python3 -c "
import sys
sys.path.append('drone')
try:
    from cluster_manager import ClusterManager
    from const import DRONES_CONFIG
    import logging
    
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger('demo')
    
    cluster_mgr = ClusterManager(logger)
    
    # Симулируем разные наборы живых дронов
    all_drones = list(DRONES_CONFIG.keys())
    test_scenarios = [
        all_drones,  # все дроны живые
        all_drones[:1] if all_drones else [],  # только первый дрон
        [],  # нет живых дронов
    ]
    
    print('Testing different cluster compositions:')
    for i, drones in enumerate(test_scenarios):
        print(f'\\nScenario {i+1}: alive_drones = {drones}')
        
        ips = cluster_mgr.get_alive_drone_ips(drones)
        print(f'  IPs: {ips}')
        
        changed = cluster_mgr.cluster_composition_changed(drones)
        print(f'  Changed: {changed}')
        
        if ips:  # Обновляем только если есть IP
            cluster_mgr.update_cluster_hosts(drones)
            current = cluster_mgr.read_current_cluster_hosts()
            print(f'  Updated cluster_hosts: {current}')
    
    print('\\nDemo completed')
    
except ImportError as e:
    print(f'Import error: {e}')
    sys.exit(1)
except Exception as e:
    print(f'Demo error: {e}')
    sys.exit(1)
"
    
    if [ $? -ne 0 ]; then
        log "ERROR: Demo failed"
        return 1
    fi
    
    log "Demo completed successfully"
    return 0
}

# Главная функция
main() {
    log "=== Cluster Configuration Test ==="
    log "Working directory: $SCRIPT_DIR"
    log "Cluster hosts file: $CLUSTER_HOSTS_FILE"
    
    local failed_tests=0
    
    # Запускаем тесты
    test_drone_config || ((failed_tests++))
    echo
    
    test_cluster_manager || ((failed_tests++))
    echo
    
    test_mpi || ((failed_tests++))
    echo
    
    test_stockfish || ((failed_tests++))
    echo
    
    demo_cluster_update || ((failed_tests++))
    echo
    
    # Итоговый отчет
    log "=== Test Summary ==="
    if [ $failed_tests -eq 0 ]; then
        log "All tests passed! ✅"
        log "Cluster setup is ready for use"
        log ""
        log "To start with cluster mode:"
        log "  export ALG_MODE=cluster"
        log "  ./run_with_cluster_restart.sh"
    else
        log "Some tests failed (${failed_tests}/5) ❌"
        log "Please fix the issues before using cluster mode"
    fi
    
    return $failed_tests
}

# Запускаем тесты
main "$@"
