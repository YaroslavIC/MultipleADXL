function composite = combineFigures(figHandles, gridSize)
    % Создает композицию из нескольких фигур
    rows = gridSize(1);
    cols = gridSize(2);
    
    % Собираем кадры из всех фигур
    frames = cell(length(figHandles), 1);
    for i = 1:length(figHandles)
        frames{i} = getframe(figHandles(i));
    end
    
    % Определяем размеры для композиции
    maxWidth = 0;
    maxHeight = 0;
    for i = 1:length(frames)
        [h, w, ~] = size(frames{i}.cdata);
        maxWidth = max(maxWidth, w);
        maxHeight = max(maxHeight, h);
    end
    
    % Создаем пустую композицию
    compositeHeight = maxHeight * rows;
    compositeWidth = maxWidth * cols;
    composite = zeros(compositeHeight, compositeWidth, 3, 'uint8');
    
    % Размещаем фигуры в композиции
    idx = 1;
    for r = 1:rows
        for c = 1:cols
            if idx <= length(frames)
                frameData = frames{idx}.cdata;
                [h, w, ~] = size(frameData);
                
                % Вычисляем позицию для вставки
                rowStart = (r-1)*maxHeight + floor((maxHeight - h)/2);
                colStart = (c-1)*maxWidth + floor((maxWidth - w)/2);
                
                % Вставляем кадр в композицию
                composite(rowStart+1:rowStart+h, colStart+1:colStart+w, :) = frameData;
                idx = idx + 1;
            end
        end
    end
end