import asyncio
from ros2launch.api.api import get_share_file_path_from_package, parse_launch_arguments
import launch

class DadsimInstance(object):
    def __init__(self):
        self._launch_service : launch.LaunchService = None
        self._launch_future = None
        self._exit_future = None
    
    def run(self):
        asyncio.create_task(self._run())
    
    async def _run(self):
        launch_file_path = get_share_file_path_from_package(package_name='dadsim', file_name='test.launch.py')
        self._launch_future = self._launch_a_launch_file_async(launch_file_path=launch_file_path, launch_file_arguments="")
        self._exit_future = asyncio.get_running_loop().create_future()
        ret = await self._launch_future
        self._exit_future.set_result(ret)
        return ret
    
    async def stop(self):
        if self._launch_service is not None:
            self._launch_service.shutdown(force_sync=True)
            return await self._exit_future
        return None

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
        launch_description = launch.LaunchDescription([
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    launch_file_path
                ),
                launch_arguments=parsed_launch_arguments,
            ),
        ])
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