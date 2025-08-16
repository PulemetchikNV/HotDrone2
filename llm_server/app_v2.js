const { createApp, ref, onMounted, nextTick } = Vue;

createApp({
    setup() {
        const messages = ref([]);
        const isLoading = ref(false);
        const messageBox = ref(null);
        let currentMessage = null;

        const scrollToBottom = () => {
            nextTick(() => {
                if (messageBox.value) {
                    messageBox.value.scrollTop = messageBox.value.scrollHeight;
                }
            });
        };

        const processLine = (line) => {
            const agentMatch = line.match(/^Agent (\S+ \(\w+\)):/);
            const iterationMatch = line.match(/^--- Итерация .* ---/);
            const finalDecisionMatch = line.match(/^--- ФИНАЛЬНОЕ РЕШЕНИЕ ---/);
            const masterChoiceMatch = line.match(/^Мастер выбрал ход: (.*)/);

            if (iterationMatch || finalDecisionMatch) {
                currentMessage = null; // End any previous message
                messages.value.push({ type: 'highlight', content: line });
            } else if (agentMatch) {
                currentMessage = { type: 'agent', agentId: agentMatch[1], content: '' };
                messages.value.push(currentMessage);
            } else if (masterChoiceMatch) {
                currentMessage = null;
                messages.value.push({ type: 'agent', agentId: 'Мастер', content: `Выбрал ход: ${masterChoiceMatch[1]}` });
            }
            else if (currentMessage && line.trim() !== '') {
                // Append line to the content of the current agent message
                currentMessage.content += line + '\n';
            } else if (line.trim() !== '') {
                // Treat as a system message if it doesn't fit other patterns
                messages.value.push({ type: 'system', content: line });
            }
            scrollToBottom();
        };

        const runScript = async () => {
            isLoading.value = true;
            messages.value = [];
            currentMessage = null;
            
            try {
                const response = await fetch('/run', { method: 'POST' });
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

        return {
            messages,
            isLoading,
            messageBox,
            runScript
        };
    }
}).mount('#app');
