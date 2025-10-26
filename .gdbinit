# GDB initialization for STM32-RTcore FreeRTOS debugging
# Only set minimal breakpoints to avoid hardware breakpoint limit (STM32F7 has 6)

# Critical breakpoints for FreeRTOS task debugging
break LED_Task
break vApplicationStackOverflowHook
break vApplicationMallocFailedHook

# Optional: Add these manually during debug session if needed
# break vTaskStartScheduler
# break prvPortStartFirstTask
# break xPortPendSVHandler
# break xPortSysTickHandler

# Display useful FreeRTOS state
# After scheduler starts, you can manually inspect:
# p pxCurrentTCB
# p xSchedulerRunning
# p/x $psp
# p/x $msp
# p/x $control

set confirm off
set history save on
set print pretty on

# Convenience command to print a short health-check for SystemInit state
define health_check
	echo --- registers ---\n
	info registers
	echo\n--- System / vector / FPU / RCC / GPIO ---\n
	p/x SystemCoreClock
	p/x *((unsigned int*)0xE000ED08)
	x/1x 0xE000ED88
	x/1x 0xE000EF34
	x/1x 0x40023830
	x/1x 0x40020414
	echo\n--- stack pointers / control ---\n
	p/x $msp
	p/x $psp
	p/x $control
	echo --- end health_check ---\n
end

# Usage:
#  source .gdbinit      # if not auto-sourced
#  health_check         # run the checks while halted (e.g. at main)
