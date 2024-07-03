import plotly.graph_objs as go

def addArrow(fig, start, vec, text=None, color='red', name='', arrow_tip_ratio=0.3, arrow_starting_ratio=0.98):
    '''
        adds an arrow to a plotly figure
        param: fig: a plotly figure (3D)
        param: start: list [x, y, z] the base location of the arrow
        param: vec: nd.array vector from start in some direction [x , y, z]
        param: text: string vector base name
        param: color: string vector color as text
        param: name: string vector name

        return fig
    '''
    vec = vec + start # vec needs to be the vectors starting from start not from the origin

    fig.add_trace(go.Scatter3d(
        x = [start[0]],
        y = [start[1]],
        z = [start[2]],
        text = text,
        mode='markers+text',
        marker=dict(
            color=color, 
            size=5,
        ),
        showlegend=False,
    ))

    x_lines = list()
    y_lines = list()
    z_lines = list()

    for i in [start, vec]:
        x_lines.append(i[0])
        y_lines.append(i[1])
        z_lines.append(i[2])
    x_lines.append(None)
    y_lines.append(None)
    z_lines.append(None)

    ## set the mode to lines to plot only the lines and not the balls/markers
    fig.add_trace(go.Scatter3d(
        x=x_lines,
        y=y_lines,
        z=z_lines,
        mode='lines',
        line = dict(width = 2, color = color),
        showlegend=False,
    ))

    name_marker = vec
    fig.add_trace(go.Scatter3d(
        x=[name_marker[0]],
        y=[name_marker[1]],
        z=[name_marker[2]],
        mode='markers+text',
        text=name, 
        marker=dict(
            color=color, 
            size=0.001,
        ),
        showlegend=False,
    ))


    ## the cone will point in the direction of vector field u, v, w 
    ## so we take this to be the difference between each pair 

    ## then hack the colorscale to force it to display the same color
    ## by setting the starting and ending colors to be the same

    fig.add_trace(go.Cone(
        x=[start[0] + arrow_starting_ratio*(vec[0] - start[0])],
        y=[start[1] + arrow_starting_ratio*(vec[1] - start[1])],
        z=[start[2] + arrow_starting_ratio*(vec[2] - start[2])],
        u=[arrow_tip_ratio*(vec[0] - start[0])],
        v=[arrow_tip_ratio*(vec[1] - start[1])],
        w=[arrow_tip_ratio*(vec[2] - start[2])],
        showlegend=False,
        showscale=False,
        colorscale=[[0, color], [1, color]]
        ))
    
    fig.update_layout(
        width=1024,
        height=1024,
        scene = dict(
            xaxis=dict(),
            yaxis=dict(),
            zaxis=dict(),
            aspectmode='data', #this string can be 'data', 'cube', 'auto', 'manual'
            #a custom aspectratio is defined as follows:
            aspectratio=dict(x=1, y=1, z=1)
        ), 
        scene_camera = dict(
            up=dict(x=0, y=-0, z=20),
            center=dict(x=0, y=0, z=0),
            eye=dict(x=-2, y=-0.5, z=3)
            )
    )

    return fig

def addMarker(fig, pos, color='blue', name=''):
    '''
        adds a marker to the given figure at the given location
        param: fig: a plotly figure (3d)
        param: pos: nd.array  list with the marker corrdinates in the plotly figure
        param: color: string marker color as text
        param: name: string amrker name as text

        return plotly figure with added marker
    '''
    fig.add_trace(go.Scatter3d(
            x=[pos[0]],
            y=[pos[1]],
            z=[pos[2]],
            mode='markers+text',
            text=name,
            marker=dict(
                size=5,
                color=color,
            )
    ))

    fig.update_layout(
        width=1024,
        height=1024,
        scene = dict(
            xaxis=dict(),
            yaxis=dict(),
            zaxis=dict(),
            aspectmode='data', #this string can be 'data', 'cube', 'auto', 'manual'
            #a custom aspectratio is defined as follows:
            aspectratio=dict(x=1, y=1, z=1)
        ), 
        scene_camera = dict(
            up=dict(x=0, y=-0, z=20),
            center=dict(x=0, y=0, z=0),
            eye=dict(x=-2, y=-0.5, z=3)
            )
    )

    return fig