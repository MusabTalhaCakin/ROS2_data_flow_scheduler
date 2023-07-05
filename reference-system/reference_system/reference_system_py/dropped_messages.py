# Copyright 2021 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import random

# from bokeh.layouts import layout
from bokeh.models import ColumnDataSource
from bokeh.models import DatetimeTickFormatter
from bokeh.models.ranges import FactorRange
from bokeh.models.tools import HoverTool
from bokeh.models.widgets.markups import Div
from bokeh.models.widgets.tables import DataTable, TableColumn
from bokeh.palettes import cividis
from bokeh.plotting import figure
from bokeh.transform import factor_cmap
try:
    import networkx as nx
    nx_available = True
except ImportError:
    nx_available = False

import numpy as np
import pandas as pd


def individual(data_model, size):
    data_dict = parseData(data_model)
    dropped_df = data_dict['dropped']
    latency_df = data_dict['latency']
    run_time = data_dict['run_time']

    y = list(zip(dropped_df.node, dropped_df.topic))

    latency_source = ColumnDataSource(latency_df)
    dropped_source = ColumnDataSource(dropped_df)

    dropped_source.data['y'] = y
    # use this for axis of figure ~0.25 of buffer
    max_dropped = max(dropped_df['dropped']) + 0.25
    # initialize figure
    dropped_fig = figure(
        title='Dropped Messages Summary ({:.2f} s) [UNDER DEVELOPMENT]'.format(float(run_time)),
        y_axis_label='Callback by Node and Topic Name',
        x_axis_label='Dropped Messages',
        y_range=FactorRange(*y),
        x_range=(0, max_dropped),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10),
        sizing_mode='scale_width'
    )
    # add horizontal bar to figure
    dropped_fig.hbar(
        y='y',
        right='dropped',
        width=0.1,
        source=dropped_source,
        fill_color=factor_cmap(
            'y',
            palette=cividis(dropped_df['topic'].shape[0]),
            factors=list(dropped_df['topic'].values.tolist()),
            start=1, end=2)
    )
    dropped_fig.yaxis.major_label_orientation = 'horizontal'
    dropped_fig.yaxis.subgroup_label_orientation = 'horizontal'
    dropped_fig.yaxis.group_label_orientation = 'horizontal'
    # add hover tool
    hover = HoverTool()
    hover.tooltips = [
        ('Callback (Node and Topic)', '@node @topic'),
        ('Dropped', '@dropped'),
        ('Expected', '@expected_count'),
        ('Received', '@count')
    ]
    dropped_fig.add_tools(hover)

    # add dropped messages table
    dropped_summary = dropped_df.describe().T.reset_index()
    columns = [TableColumn(field=col, title=col) for col in dropped_summary]
    dropped_table_title = Div(
        text='<b>Dropped Messages Summary Table [UNDER DEVELOPMENT]</b>',
        width=1000,
        height=10
    )
    dropped_table = [
        dropped_table_title,
        DataTable(
            columns=columns,
            source=ColumnDataSource(dropped_summary),
            margin=(10, 10, 10, 10),
            height=150,
            width=size)]

    # add latency plot
    latency_fig = figure(
        title='Latency From Front Lidar to Collision Estimator [UNDER DEVELOPMENT]',
        x_axis_label='Time',
        y_axis_label='Latency to Object Collision Estimator (ms)',
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10),
        sizing_mode='scale_width'
    )
    latency_fig.line(
        x='timestamp',
        y='latency',
        source=latency_source,
        line_width=2
    )
    latency_fig.xaxis[0].formatter = DatetimeTickFormatter(seconds=['%Ss'])

    # add latency table
    latency_summary = latency_df.describe().T.reset_index()
    columns = [TableColumn(field=col, title=col) for col in latency_summary]
    latency_table_title = Div(
        text='<b>Latency Summary Table</b>',
        width=1000,
        height=10
    )
    latency_table = [
        latency_table_title,
        DataTable(
            columns=columns,
            source=ColumnDataSource(latency_summary),
            margin=(10, 10, 10, 10),
            height=75,
            width=size)]
    return [
        [dropped_table], [dropped_fig],
        [latency_table], [latency_fig]
    ]


def parseData(data_model):
    callback_symbols = data_model.get_callback_symbols()
    colors = []  # Adds random colors for each callback
    color_i = 0
    earliest_date = None
    latest_date = None
    dropped_data = []
    front_lidar_data = pd.DataFrame()
    object_collision_data = pd.DataFrame()
    for obj, symbol in callback_symbols.items():
        callback_df = data_model.get_callback_durations(obj)
        # get node information and filter out internal subscriptions
        owner_info = data_model.get_callback_owner_info(obj)
        if owner_info is not None:
            if '/parameter_events' in owner_info:
                continue
            period = 0.0
            if 'period' in owner_info:
                # assume in milliseconds so convert to seconds
                period = float(owner_info[
                    owner_info.find('period: ') + len('period: '):owner_info.rfind(' ')]) / 1000
            if 'FrontLidarDriver' in owner_info and 'Timer' in owner_info:
                front_lidar_data = callback_df
            if 'ObjectCollisionEstimator' in owner_info:
                object_collision_data = callback_df

        # add color to list if needed
        if(len(colors) <= color_i):
            colors.append('#%06X' % random.randint(0, 256**3-1))
        # get human readable name of callback
        substr = 'node: '
        index = str.find(owner_info, substr)
        if index >= 0:
            index += len(substr)
            node = owner_info[index:(str.find(owner_info, ','))]
        substr = 'topic: /'
        index = str.find(owner_info, substr)
        if index >= 0:
            index += len(substr)
            topic = owner_info[index:]
        else:
            topic = ''
        # get first and last timestamp of data
        thefirstdate = callback_df.loc[:, 'timestamp'].iloc[0]
        if earliest_date is None:
            earliest_date = thefirstdate
        elif earliest_date is not None and thefirstdate <= earliest_date:
            earliest_date = thefirstdate
        thelastdate = callback_df.loc[:, 'timestamp'].iloc[len(callback_df)-1]
        if latest_date is None:
            latest_date = thelastdate
        elif latest_date is not None and thelastdate >= latest_date:
            latest_date = thelastdate
        # add name of callback and count to list
        dropped_data.append(
            [str(node), str(topic), float(len(callback_df)), period, 0.0, 0.0, colors[color_i]])
        color_i += 1

    front_lidar_data = front_lidar_data.reset_index(drop=True)
    object_collision_data = object_collision_data.reset_index(drop=True)

    # ensure dataframes are same length for latency calc, drop extra lidar frames
    extra = len(front_lidar_data) - len(object_collision_data)
    if(extra > 0):
        front_lidar_data.drop(front_lidar_data.tail(extra).index, inplace=True)
    latency = object_collision_data['timestamp'] - front_lidar_data['timestamp']
    dropped_df = pd.DataFrame(
        dropped_data,
        columns=['node', 'topic', 'count', 'period', 'dropped', 'expected_count', 'color'])
    latency_df = pd.DataFrame(
        {'index': range(0, len(latency)),
         'latency': latency,
         'timestamp': front_lidar_data['timestamp']})
    # sort values by node and topic
    dropped_df = dropped_df.sort_values(by=['node', 'topic'])
    # calculate run time
    approx_run_time = None
    if earliest_date is not None and latest_date is not None:
        approx_run_time = getRunTime(earliest_date, latest_date)
    # calculate estimated count and received count
    dropped_df = calcTotals(approx_run_time, dropped_df)
    node_graph = None
    if(nx_available):
        # generate node graph
        node_graph = generateNodeGraph(dropped_df)
        # count expected and dropped messages
        dropped_df = countDropped(dropped_df, node_graph)
    # prepare output
    data_dict = {
        'dropped': dropped_df,
        'latency': latency_df,
        'node_graph': node_graph,
        'start': earliest_date,
        'end': latest_date,
        'run_time': approx_run_time,
    }
    return data_dict


def getRunTime(start, end):
    # calculate run time in seconds for experiment
    return (end - start).total_seconds()


def calcTotals(run_time, dataframe):
    # calculate expected counts for each period
    mask = (dataframe['period'] != 0)
    period_non_zero = dataframe[mask]
    dataframe.loc[mask, 'expected_count'] = (run_time / period_non_zero['period']).apply(np.floor)
    return dataframe


def generateNodeGraph(dataframe):
    connections = []
    # for every node that has a defined period
    for node in dataframe.node:
        # assume node has sub nodes by default
        sub_node_exists = True
        # search for sub nodes of current node, assume node name is topic name
        current_node = node  # current top-level node of fork
        fork_topics = []
        sub_topics = []
        in_fork = False
        while sub_node_exists:
            if(len(fork_topics) != 0 and not in_fork):
                # fork topic exists
                current_node = fork_topics.pop(0)
                in_fork = True
            if(len(sub_topics) != 0):
                # sub topics still exist
                current_node = sub_topics.pop(0)
            sub_node_df = dataframe.loc[
                ((dataframe.topic == current_node) &
                 (dataframe.expected_count == 0))]
            if not sub_node_df.empty:
                # node has sub node(s)
                if(sub_node_df.shape[0] > 1):
                    # node has more than one sub node
                    for fork in sub_node_df.node:
                        if fork not in fork_topics:
                            fork_topics.append(fork)
                for sub_node in sub_node_df.node:
                    # add to list of tuples
                    connections.append((current_node, sub_node))
                    if not dataframe.loc[dataframe.topic == sub_node].empty:
                        if sub_node not in sub_topics:
                            sub_topics.append(sub_node)
                    else:
                        # no sub nodes
                        if(in_fork):
                            in_fork = False
                            continue
                        sub_node_exists = False
            else:
                # no sub nodes
                if(in_fork):
                    in_fork = False
                    continue
                sub_node_exists = False
    # generate DAG from list of tuples
    graph = nx.DiGraph()
    graph.add_edges_from(connections)
    return graph


def countDropped(dataframe, node_graph):
    mask = (dataframe['expected_count'] != 0)
    expected_non_zero = dataframe[mask]
    # for every node that has a defined period
    for node in expected_non_zero.node:
        # assume node has sub nodes by default
        sub_node_exists = True
        # search for sub nodes of current node, assume node name is topic name
        current_node = node  # current top-level node of fork
        fork_topics = []
        sub_topics = []
        in_fork = False
        # pred = len(node_graph.pred[node])
        # if(pred > 0):
        # TODO(flynneva): figure out a way to trace up the node graph
        # to determine if a fusion node reduces the total expected
        # count = expected_non_zero.expected_count.sum()
        # expected = count - dataframe.loc[(
        #     (dataframe.node == node) &
        #     (dataframe.topic == '')),
        #    'expected_count'].values[0]
        # subtract front lidar expected count due to fusion node
        # TODO(flynneva): figure out a better way to handle fusion nodes
        # expected -= dataframe.loc[(
        #    (dataframe.node == 'FrontLidarDriver') &
        #    (dataframe.topic == '')),
        #   'expected_count'].values[0]
        # else:
        # calculate dropped for current node
        expected = dataframe.loc[(
            (dataframe.node == node) &
            (dataframe.topic == '')),
            'expected_count'].values[0]
        count = dataframe.loc[(
            (dataframe.node == current_node) &
            (dataframe.topic == '')), 'count'].values[0]
        dataframe.loc[(
            (dataframe.node == current_node) &
            (dataframe.topic == '')), 'dropped'] = abs(expected - count)
        while sub_node_exists:
            if(len(fork_topics) != 0 and not in_fork):
                # fork topic exists
                current_node = fork_topics.pop(0)
                in_fork = True
            if(len(sub_topics) != 0):
                # sub topics still exist
                current_node = sub_topics.pop(0)
            sub_node_df = dataframe.loc[
                ((dataframe.topic == current_node) &
                 (dataframe.expected_count == 0))]
            if not sub_node_df.empty:
                # node has sub node(s)
                if(sub_node_df.shape[0] > 1):
                    # node has more than one sub node
                    for fork in sub_node_df.node:
                        if fork not in fork_topics:
                            fork_topics.append(fork)
                for sub_node in sub_node_df.node:
                    dataframe.loc[(
                        (dataframe.node == sub_node) &
                        (dataframe.topic == current_node)), 'expected_count'] = expected
                    count = dataframe.loc[(
                        (dataframe.node == sub_node) &
                        (dataframe.topic == current_node)), 'count'].values[0]
                    dataframe.loc[(
                        (dataframe.node == sub_node) &
                        (dataframe.topic == current_node)), 'dropped'] = abs(expected - count)
                    if not dataframe.loc[dataframe.topic == sub_node].empty:
                        if sub_node not in sub_topics:
                            sub_topics.append(sub_node)
                    else:
                        # no sub nodes
                        if(in_fork):
                            in_fork = False
                            continue
                        sub_node_exists = False
            else:
                # no sub nodes
                if(in_fork):
                    in_fork = False
                    continue
                sub_node_exists = False
    return dataframe
