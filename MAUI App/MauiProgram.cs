using Microcharts.Maui;
using Microsoft.Extensions.Logging;

namespace APM_Running_App
{
    public static class MauiProgram
    {
        public static MauiApp CreateMauiApp()
        {
            var builder = MauiApp.CreateBuilder();
            builder
                .UseMauiApp<App>()
                .UseMicrocharts()
                .ConfigureFonts(fonts =>
                {
                    fonts.AddFont("OpenSans-Regular.ttf", "OpenSansRegular");
                    fonts.AddFont("OpenSans-Semibold.ttf", "OpenSansSemibold");
                });

            builder.Services.AddSingleton<MainPage>();
            builder.Services.AddTransient<SteadyPacePage>();
            builder.Services.AddTransient<UserDefinedPage>();
            builder.Services.AddTransient<PatternElitePage>();
            builder.Services.AddTransient<IntervalPage>();
            builder.Services.AddTransient<InfoPage>();
            builder.Services.AddTransient<SettingsPage>();
#if DEBUG
            builder.Logging.AddDebug();
#endif

            return builder.Build();
        }
    }
}
