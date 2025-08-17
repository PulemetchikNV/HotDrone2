const { createApp, ref, onMounted, nextTick } = Vue;

createApp({
    setup() {
        const messages = ref([]);
        const isLoading = ref(false);
        const messageBox = ref(null);
        const recentMoves = ref([]);
        const isPolling = ref(false);
        const connectionStatus = ref('disconnected');
        const lastPollTimestamp = ref(null);
        const pollInterval = ref(null);
        let currentMessage = null;

        const avatarMap = {
            'king': '0.png',
            'queen': '1.png',
            'bishop': '2.png',
            'knight': '3.png',
            'pawn': '4.png',
            'rook': '4.png', // Assuming Rook uses the same icon as Pawn for now
            'master': '0.png' // Master uses the King's avatar
        };
        
        const russianRoles = {
            "king": "Король", "queen": "Королева", "bishop": "Слон", "knight": "Конь", "rook": "Ладья", "pawn": "Пешка",
            "master": "Мастер"
        };

        const scrollToBottom = () => {
            nextTick(() => {
                if (messageBox.value) {
                    messageBox.value.scrollTop = messageBox.value.scrollHeight;
                }
            });
        };

        const pollForMoves = async () => {
            if (isPolling.value) return;
            
            try {
                isPolling.value = true;
                connectionStatus.value = 'connecting';
                
                const url = lastPollTimestamp.value 
                    ? `http://192.168.1.119:8080/poll?since=${encodeURIComponent(lastPollTimestamp.value)}`
                    : 'http://192.168.1.119:8080/poll';
                    
                const response = await fetch(url);
                const data = await response.json();
                
                if (data.status === 'success') {
                    connectionStatus.value = 'connected';
                    lastPollTimestamp.value = data.timestamp;
                    
                    // Обновляем список ходов
                    if (data.moves && data.moves.length > 0) {
                        recentMoves.value = [...recentMoves.value, ...data.moves];
                        // Оставляем только последние 20 ходов
                        recentMoves.value = recentMoves.value.slice(-20);
                        
                        // Добавляем новые ходы в чат
                        data.moves.forEach(move => {
                            messages.value.push({
                                type: 'move',
                                timestamp: move.timestamp,
                                content: `🚁 Новый ход: ${move.move} (${move.from_cell} → ${move.to_cell})`,
                                engine: move.engine,
                                fen: move.fen
                            });
                        });
                        
                        scrollToBottom();
                    }
                } else {
                    connectionStatus.value = 'error';
                }
            } catch (error) {
                console.error('Polling error:', error);
                connectionStatus.value = 'error';
            } finally {
                isPolling.value = false;
            }
        };

        const startPolling = () => {
            if (pollInterval.value) return;
            
            pollInterval.value = setInterval(pollForMoves, 2000); // Каждые 2 секунды
            pollForMoves(); // Первый запрос сразу
        };

        const stopPolling = () => {
            if (pollInterval.value) {
                clearInterval(pollInterval.value);
                pollInterval.value = null;
            }
            connectionStatus.value = 'disconnected';
        };

        const processLine = (line) => {
            const [type, ...parts] = line.split(':');
            const content = parts.join(':');

            switch (type) {
                case 'FIGURE':
                    const [figure, role, thought] = parts;
                    const agentName = russianRoles[figure] || figure;
                    messages.value.push({
                        type: 'agent',
                        agentId: `${agentName} (${role})`,
                        avatar: avatarMap[figure] || '4.png',
                        content: thought
                    });
                    break;
                case 'MASTER':
                    messages.value.push({
                        type: 'agent',
                        agentId: 'Мастер',
                        avatar: avatarMap['master'],
                        content: content
                    });
                    break;
                case 'SYSTEM':
                    messages.value.push({ type: 'highlight', content: content });
                    break;
                case 'FINAL_MOVE':
                     messages.value.push({ type: 'highlight', content: `--- ФИНАЛЬНОЕ РЕШЕНИЕ ---` });
                     messages.value.push({
                        type: 'agent',
                        agentId: 'Мастер',
                        avatar: avatarMap['master'],
                        content: `Выбранный ход: ${content}`
                    });
                    break;
                case 'ERROR':
                    messages.value.push({ type: 'system', content: `ОШИБКА: ${content}` });
                    break;
                default:
                     if (line.trim()) {
                        messages.value.push({ type: 'system', content: line });
                    }
            }
            scrollToBottom();
        };

        const togglePolling = () => {
            if (pollInterval.value) {
                stopPolling();
            } else {
                startPolling();
            }
        };

        const runScript = async () => {
            isLoading.value = true;
            messages.value = [];
            currentMessage = null;
            
            try {
                const response = await fetch('http://192.168.1.119:8080/run', { method: 'POST' });
                const reader = response.body.getReader();
                const decoder = new TextDecoder();

                while (true) {
                    const { done, value } = await reader.read();
                    if (done) break;

                    const chunk = decoder.decode(value, { stream: true });
                    const lines = chunk.split('\n');
                    
                    lines.forEach(line => {
                        if (line.startsWith('data: ')) {
                            const data = line.substring(6).trim();
                            if (data) {
                                processLine(data);
                            }
                        }
                    });
                }
            } catch (error) {
                console.error('Error running script:', error);
                messages.value.push({ type: 'system', content: 'Ошибка: Не удалось выполнить скрипт.' });
            } finally {
                isLoading.value = false;
                currentMessage = null;
                scrollToBottom();
            }
        };

        // Автозапуск polling при загрузке страницы
        onMounted(() => {
            startPolling();
        });

        return {
            messages,
            isLoading,
            messageBox,
            recentMoves,
            connectionStatus,
            isPolling,
            runScript,
            togglePolling,
            startPolling,
            stopPolling,
            pollInterval
        };
    }
}).mount('#app');
