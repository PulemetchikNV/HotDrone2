const { createApp, ref, onMounted, nextTick } = Vue;

createApp({
    setup() {
        const messages = ref([]);
        const isLoading = ref(false);
        const messageBox = ref(null);

        const scrollToBottom = () => {
            nextTick(() => {
                if (messageBox.value) {
                    messageBox.value.scrollTop = messageBox.value.scrollHeight;
                }
            });
        };

        const parseLog = (logText) => {
            const lines = logText.split('\n\n');
            let messageId = 0;
            const parsedMessages = [];

            lines.forEach(block => {
                if (block.trim() === '') return;

                const agentMatch = block.match(/^Agent (\S+) \((\w+)\):/);
                const masterDecisionMatch = block.match(/^--- .* Решение Мастера ---/);
                const systemMessageMatch = block.match(/^--- .* ---/);
                const finalDecisionMatch = block.match(/^--- ФИНАЛЬНОЕ РЕШЕНИЕ ---/);

                if (agentMatch) {
                    const agentId = agentMatch[1];
                    const role = agentMatch[2];
                    const content = block.substring(agentMatch[0].length).trim();
                    parsedMessages.push({ id: messageId++, type: 'agent', role, agentId, content });
                } else if (masterDecisionMatch) {
                    const content = block.split('\n').slice(1).join('\n').trim();
                    parsedMessages.push({ id: messageId++, type: 'agent', role: 'master', agentId: 'Главный агент', content });
                } else if (finalDecisionMatch) {
                    const content = block.trim();
                     parsedMessages.push({ id: messageId++, type: 'system', content });
                }
                else if (systemMessageMatch) {
                    const content = block.trim();
                    parsedMessages.push({ id: messageId++, type: 'system', content });
                } else {
                    // Fallback for initial state or other text
                    if (!block.startsWith('Initial board state')) {
                         parsedMessages.push({ id: messageId++, type: 'system', content: block.trim() });
                    }
                }
            });
            return parsedMessages;
        };

        const fetchLog = async () => {
            try {
                const response = await fetch('/log');
                if (response.ok) {
                    const data = await response.text();
                    messages.value = parseLog(data);
                } else {
                    messages.value = [{ id: 0, type: 'system', content: 'Ошибка при загрузке лога.' }];
                }
            } catch (error) {
                console.error('Error fetching log:', error);
                messages.value = [{ id: 0, type: 'system', content: 'Не удалось подключиться к серверу.' }];
            }
            scrollToBottom();
        };

        const runScript = async () => {
            isLoading.value = true;
            messages.value = [{ id: 0, type: 'system', content: 'Запуск скрипта...' }];
            try {
                const response = await fetch('/run', { method: 'POST' });
                if (response.ok) {
                    const result = await response.text();
                    messages.value = parseLog(result);
                } else {
                    const errorText = await response.text();
                    messages.value = [{ id: 0, type: 'system', content: `Ошибка при выполнении скрипта:\n${errorText}` }];
                }
            } catch (error) {
                console.error('Error running script:', error);
                messages.value = [{ id: 0, type: 'system', content: 'Не удалось выполнить скрипт. Убедитесь, что сервер запущен.' }];
            } finally {
                isLoading.value = false;
                scrollToBottom();
            }
        };

        onMounted(() => {
            fetchLog();
        });

        return {
            messages,
            isLoading,
            messageBox,
            runScript
        };
    }
}).mount('#app');
