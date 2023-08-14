function vizFormation(adjacencyMatrix)
    % chatGPT
    % Get the number of nodes in the graph
    numNodes = size(adjacencyMatrix, 1);

    % Calculate the positions of nodes in a regular polygon
    angle = 2 * pi / numNodes;
    radius = 1;
    theta = 0:angle:2 * pi;
    x = radius * cos(theta);
    y = radius * sin(theta);

    % Plot the nodes as markers 'o'
    figure;
    hold on;
    for i = 1:numNodes
        plot(x(i), y(i), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        text(x(i), y(i), num2str(i), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end

    % Plot the edges based on the adjacency matrix
    for i = 1:numNodes
        for j = i+1:numNodes
            if adjacencyMatrix(i, j) == 1
                plot([x(i), x(j)], [y(i), y(j)], 'k');
            end
        end
    end

    axis equal;
    title('Graph with Regular Polygon Arrangement');
    hold off;
end

