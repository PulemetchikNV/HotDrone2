const { createApp, ref, onMounted, nextTick } = Vue;

createApp({
    setup() {
        const messages = ref([]);
        const isLoading = ref(false);
        const messageBox = ref(null);
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
