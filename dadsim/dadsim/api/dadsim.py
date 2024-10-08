import asyncio
from ros2launch.api.api import get_share_file_path_from_package, parse_launch_arguments
import launch
from launch.launch_context import LaunchContext
from launch.events.process.process_exited import ProcessExited
from launch.event_handlers import OnProcessExit
from launch_ros.actions import PushRosNamespace
import launch.logging
import logging
import logging.handlers
import os

class DadsimInstance(object):
    def __init__(self, launch_file_path=None, quiet=False, namespace=None):
        self._launch_service : launch.LaunchService = None
        self._launch_future = None
        self._exit_future = None
        self._launch_file_path = get_share_file_path_from_package(package_name='dadsim', file_name='test.launch.py') if launch_file_path is None else launch_file_path
        self._running = False
        self._launch_task = None
        self._namespace = namespace
        
        if quiet:
            os.environ['DADSIM_QUIET'] = 'True'
            screen_handler = logging.NullHandler()
            launch.logging.launch_config.screen_handler = screen_handler
        else:
            os.environ['DADSIM_QUIET'] = 'False'
    
    def exception(self):
        try:
            return self._launch_task.exception()
        # TODO: Handle cancel and uncomplete
        except asyncio.exceptions.InvalidStateError:
            return None
        except asyncio.exceptions.CancelledError:
            return None
    
    @property
    def running(self):
        return self._running
    
    def run(self, arguments=[]):
        self._running = True
        self._launch_task = asyncio.create_task(self._run(arguments=arguments))
    
    async def _run(self, arguments=[]):
        
        self._launch_future = asyncio.gather(self._launch_a_launch_file_async(launch_file_path=self._launch_file_path, launch_file_arguments=arguments))
        self._exit_future = asyncio.get_running_loop().create_future()
        ret = None
        ret = await self._launch_future
        self._exit_future.set_result(ret)
        return ret
    
    async def stop(self):
        self._running = False
        if self._launch_service is not None:
            self._launch_service.shutdown(force_sync=True)
            await self._exit_future
        await self._launch_future
        return None
    
    # def process_exit_handler(self, event:ProcessExited, context:LaunchContext):
    #     if self._running:
    #         asyncio.get_running_loop().call_soon_threadsafe(self._launch_future.set_exception, self.ProcessExitException())

    async def _launch_a_launch_file_async(
        self,
        *,
        launch_file_path,
        launch_file_arguments,
        noninteractive=False,
        args=None,
        option_extensions={},
        debug=False
    ):
        """Launch a given launch file (by path) and pass it the given launch file arguments."""
        for name in sorted(option_extensions.keys()):
            option_extensions[name].prestart(args)
        self._launch_service = launch.LaunchService(
            argv=launch_file_arguments,
            noninteractive=noninteractive,
            debug=debug)
        parsed_launch_arguments = parse_launch_arguments(launch_file_arguments)
        # Include the user provided launch file using IncludeLaunchDescription so that the
        # location of the current launch file is set.
        include_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.AnyLaunchDescriptionSource(
                launch_file_path
            ),
            launch_arguments=parsed_launch_arguments,
        )
        launch_description = launch.LaunchDescription([
            PushRosNamespace(namespace=self._namespace),
            include_launch
        ] if self._namespace is not None else [
            include_launch
        ])
        launch_description.add_action(launch.actions.RegisterEventHandler(
            event_handler=OnProcessExit(
                # on_exit=self.process_exit_handler
                on_exit=launch.actions.Shutdown(reason="Unexpected process exit")
            )
        ))
        for name in sorted(option_extensions.keys()):
            result = option_extensions[name].prelaunch(
                launch_description,
                args
            )
            launch_description = result[0]
        self._launch_service.include_launch_description(launch_description)
        ret = await self._launch_service.run_async()
        for name in sorted(option_extensions.keys()):
            option_extensions[name].postlaunch(ret, args)
        return ret

    class ProcessExitException(BaseException):
        """A process exited unexpectedly."""

async def main():
    instance = DadsimInstance()
    for i in range(2):
        print(f"Running {i}")
        instance.run()
        await asyncio.sleep(10)
        ret = await instance.stop()
        print(f"Stopped {ret}")
    
if __name__ == '__main__':
    asyncio.run(main())