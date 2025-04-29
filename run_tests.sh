#!/bin/bash

# Скрипт для запуска тестов и расчета средних показателей

# Директория с тестовыми файлами
TEST_DIR="../instances/den312d/100_agents"

total_soc=0
total_makespan=0
total_tests=0


for i in {1..50}; do
    test_file="${TEST_DIR}/test${i}.txt"
    
    if [ ! -f "$test_file" ]; then
        echo "Файл $test_file не найден, пропускаем..."
        continue
    fi
    
    output=$(./app -i "$test_file" -s FlowNetwork -v 2>&1)
    
    soc=$(echo "$output" | grep -oP 'soc=\s*\K\d+')
    makespan=$(echo "$output" | grep -oP 'makespan=\s*\K\d+')
    
    if [ -n "$soc" ] && [ -n "$makespan" ]; then
        total_soc=$((total_soc + soc))
        total_makespan=$((total_makespan + makespan))
        total_tests=$((total_tests + 1))
        echo "Test $i: soc=$soc, makespan=$makespan"
    else
        echo "Test $i: не удалось получить результаты"
    fi
done

if [ $total_tests -gt 0 ]; then
    soc_average=$((total_soc / total_tests))
    makespan_average=$((total_makespan / total_tests))
    
    echo "----------------------------------------"
    echo "Всего успешных тестов: $total_tests"
    echo "Средний SOC: $soc_average"
    echo "Средний Makespan: $makespan_average"
    echo "Общий SOC: $total_soc"
    echo "Общий Makespan: $total_makespan"
else
    echo "Нет успешных тестов для расчета"
fi
